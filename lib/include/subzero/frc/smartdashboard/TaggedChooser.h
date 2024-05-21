#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "subzero/frc/smartdashboard/ModifiableChooser.h"

namespace subzero {

/**
 * @brief Each key of type T has a vector<string> of tags; accepts a list of
 * groups that each have a name and list of possible tags that are mutually
 * exclusive. The ANY option is automatically included in all group selectors to
 * indicate the lack of a selection. When a selection is made, the intersection
 * of all selections is created based on T's list of tags
 *
 * @tparam TKey
 */
template <typename TKey> class TaggedChooser {
public:
  /**
   * @brief A pair composed of the key and its name
   *
   */
  using TaggedChooserValue = std::pair<TKey, std::string>;
  /**
   * @brief A pair composed of TaggedChooserValue and a set of associated tags
   *
   */
  using TaggedChooserEntry =
      std::pair<TaggedChooserValue, std::set<std::string>>;
  /**
   * @brief Describes a single group which has a name and a set of available
   * tags from which to choose
   *
   */
  using TaggedChooserSelectorGroup =
      std::pair<std::string, std::set<std::string>>;

  /**
   * @brief Construct a new TaggedChooser
   *
   * @param entries List of filterable entries
   * @param groups List of group selectors
   * @param chooserName Name of the TaggedChooser in SmartDashboard
   */
  TaggedChooser(const std::vector<TaggedChooserEntry> &entries,
                const std::vector<TaggedChooserSelectorGroup> &groups,
                std::string chooserName)
      : m_chooserName{chooserName} {
    m_entries = entries;
    m_groups.reserve(groups.size());

    for (auto &group : groups) {
      m_groups.push_back({
          .group = group,
          .chooser = std::make_unique<frc::SendableChooser<std::string>>(),
      });
    }

    m_chooser.OnChange([this](std::optional<TKey> value) {
      if (value && m_onChangeCb) {
        m_onChangeCb(value.value());
      }
    });

    frc::SmartDashboard::PutData(m_chooserName, &m_chooser);
  }

  /**
   * @brief Call this one on startup to populate and push to SmartDashboard
   *
   */
  void Initialize() {
    for (auto &group : m_groups) {
      for (auto &option : group.group.second) {
        group.chooser->AddOption(option, option);
      }

      group.chooser->SetDefaultOption("ANY", "ANY");
      group.chooser->OnChange([this](std::string newValue) {
        auto availableEntries = GetAvailableEntries();
        PopulateChooser();
        std::vector<TKey> availableKeys;
        availableKeys.reserve(availableEntries.size());

        std::transform(
            availableEntries.begin(), availableEntries.end(),
            std::back_inserter(availableKeys),
            [](const TaggedChooserValue &value) { return value.first; });
      });

      frc::SmartDashboard::PutData(group.group.first, group.chooser.get());
    }

    PopulateChooser();
  }

  /**
   * @brief Register a callback that gets executed each time the selected option
   * changes
   *
   * @param cb
   */
  void SetOnChangeCallback(std::function<void(TKey)> cb) { m_onChangeCb = cb; }

  /**
   * @brief Get the list of all available entries in the selector
   *
   * @return std::vector<TaggedChooserValue> List of entries
   */
  std::vector<TaggedChooserValue> GetAvailableEntries() {
    std::vector<TaggedChooserValue> availableEntries;
    std::vector<std::string> selectedTags;
    for (auto &group : m_groups) {
      auto selected = group.chooser->GetSelected();
      if (selected != "ANY") {
        selectedTags.push_back(selected);
      }
    }

    for (auto &entry : m_entries) {
      bool matches = true;
      for (auto &tag : selectedTags) {
        if (!entry.second.contains(tag)) {
          matches = false;
          break;
        }
      }

      if (matches) {
        availableEntries.push_back(entry.first);
      }
    }

    return availableEntries;
  }

  /**
   * @brief Repopulate the chooser based on the selected group filters
   *
   */
  void PopulateChooser() {
    auto entries = GetAvailableEntries();
    m_chooser.ClearOptions();

    for (auto it = entries.begin(); it != entries.end(); it++) {
      if (it == entries.begin()) {
        m_chooser.SetDefaultOption(it->second, it->first);
        continue;
      }

      m_chooser.AddOption(it->second, it->first);
    }
  }

  /**
   * @brief Get the currently-selected option
   *
   * @return TKey
   */
  inline TKey GetSelectedValue() { return m_chooser.GetSelected(); }

private:
  struct TaggedChooserSendableGroup {
    TaggedChooserSelectorGroup group;
    std::unique_ptr<frc::SendableChooser<std::string>> chooser;
  };
  std::function<void(TKey)> m_onChangeCb;
  std::vector<TaggedChooserEntry> m_entries;
  std::vector<TaggedChooserSendableGroup> m_groups;
  ModifiableChooser<TKey> m_chooser;
  std::string m_chooserName;
};
}