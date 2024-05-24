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

#include "subzero/frc/smartdashboard/ModifiableChooser.cpp"

namespace subzero
{

  /**
   * @brief Each key of type T has a vector<string> of tags; accepts a list of
   * groups that each have a name and list of possible tags that are mutually
   * exclusive. The ANY option is automatically included in all group selectors to
   * indicate the lack of a selection. When a selection is made, the intersection
   * of all selections is created based on T's list of tags
   *
   * @tparam TKey
   */
  template <typename TKey>
  class TaggedChooser
  {
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
                  std::string chooserName);

    /**
     * @brief Call this one on startup to populate and push to SmartDashboard
     *
     */
    void Initialize(void);

    /**
     * @brief Register a callback that gets executed each time the selected option
     * changes
     *
     * @param cb
     */
    void SetOnChangeCallback(std::function<void(TKey)> cb);

    /**
     * @brief Get the list of all available entries in the selector
     *
     * @return std::vector<TaggedChooserValue> List of entries
     */
    std::vector<TaggedChooserValue> GetAvailableEntries(void);

    /**
     * @brief Repopulate the chooser based on the selected group filters
     *
     */
    void PopulateChooser(void);

    /**
     * @brief Get the currently-selected option
     *
     * @return TKey
     */
    inline TKey GetSelectedValue() { return m_chooser.GetSelected(); }

  private:
    struct TaggedChooserSendableGroup
    {
      TaggedChooserSelectorGroup group;
      std::unique_ptr<frc::SendableChooser<std::string>> chooser;
    };
    std::function<void(TKey)> m_onChangeCb;
    std::vector<TaggedChooserEntry> m_entries;
    std::vector<TaggedChooserSendableGroup> m_groups;
    ModifiableChooser<TKey> m_chooser;
    std::string m_chooserName;
  };
} // namespace subzero