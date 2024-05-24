#pragma once

#include "subzero/frc/smartdashboard/TaggedChooser.h"

using namespace subzero;

template <typename T>
TaggedChooser<T>::TaggedChooser(
    const std::vector<TaggedChooserEntry> &entries,
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

  m_chooser.OnChange([this](std::optional<T> value) {
    if (value && m_onChangeCb) {
      m_onChangeCb(value.value());
    }
  });

  frc::SmartDashboard::PutData(m_chooserName, &m_chooser);
}

template <typename T>
void TaggedChooser<T>::SetOnChangeCallback(std::function<void(T)> cb) {
  m_onChangeCb = cb;
}

template <typename T> void TaggedChooser<T>::Initialize() {
  for (auto &group : m_groups) {
    for (auto &option : group.group.second) {
      group.chooser->AddOption(option, option);
    }

    group.chooser->SetDefaultOption("ANY", "ANY");
    group.chooser->OnChange([this](std::string newValue) {
      auto availableEntries = GetAvailableEntries();
      PopulateChooser();
      std::vector<T> availableKeys;
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

template <typename T>
std::vector<typename TaggedChooser<T>::TaggedChooserValue>
TaggedChooser<T>::GetAvailableEntries() {
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

template <typename T> void TaggedChooser<T>::PopulateChooser() {
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