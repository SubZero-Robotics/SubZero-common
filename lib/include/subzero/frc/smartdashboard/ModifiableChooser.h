// See
// https://github.com/frc1675/frc1675-2024/blob/42d94881e7dd001fac9bb410892267b4d4dd8063/src/main/java/frc/robot/util/ChangableSendableChooser.java

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include <atomic>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <ranges>
#include <string>
#include <vector>

namespace subzero {

/**
 * @brief A SmartDashboard drop-down that can have its options dynamically
 * change
 *
 * @tparam T The underlying value to return for a selected option
 */
template <typename T> class ModifiableChooser : public wpi::Sendable {
private:
  static inline const std::string kDefault = "default";
  static inline const std::string kSelected = "selected";
  static inline const std::string kActive = "active";
  static inline const std::string kOptions = "options";
  static inline const std::string kInstance = ".instance";

  std::map<std::string, T> m_map;

  std::string m_defaultChoice = "";
  int m_instance;
  std::string m_previousValue;
  std::function<void(std::optional<T>)> m_listener;
  std::atomic_int m_instances{0};
  std::string m_selected;
  std::recursive_mutex m_mutex;

public:
  ModifiableChooser();

  ~ModifiableChooser();

  /**
   * @brief Add a new option to the chooser
   *
   * @param name Name to display
   * @param object Value that gets returned upon selection
   */
  void AddOption(std::string name, T object);

  /**
   * @brief Removes the option and updates the default option accoringly
   *
   * @param name
   */
  void RemoveOption(std::string name);

  /**
   * @brief Clears all options
   *
   */
  void ClearOptions();

  /**
   * @brief Populates options from an existing set
   *
   * @param options
   */
  void SetOptions(std::map<std::string, T> options);

  /**
   * @brief Set the default option to return
   *
   * @param name
   * @param object
   */
  void SetDefaultOption(std::string name, T object);

  /**
   * @brief Set the options from an existing set along with a default
   *
   * @param options
   * @param defaultName
   * @param defaultObject
   */
  void SetOptions(std::map<std::string, T> options, std::string defaultName,
                  T defaultObject);

  /**
   * @brief Get the selected option
   *
   * @return T
   */
  T GetSelected();

  /**
   * @brief Get the selected key rather than the value
   *
   * @return std::string
   */
  std::string GetSelectedKey();

  std::string GetNtSelected();

  void SetNtSelected(std::string val);

  /**
   * @brief Register a callback that gets executed whenever the selection
   * changes
   *
   * @param listener
   */
  void OnChange(std::function<void(std::optional<T>)> listener);

  void InitSendable(wpi::SendableBuilder &builder) override;
};
} // namespace subzero