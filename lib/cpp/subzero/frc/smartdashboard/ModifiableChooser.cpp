#pragma once

#include "subzero/frc/smartdashboard/ModifiableChooser.h"

using namespace subzero;

template <typename T>
ModifiableChooser<T>::ModifiableChooser()
{
    m_instance = m_instances.fetch_add(1);
    wpi::SendableRegistry::Add(this, "SendableChangableChooser", m_instance);
}

template <typename T>
ModifiableChooser<T>::~ModifiableChooser()
{
    wpi::SendableRegistry::Remove(this);
}

template <typename T>
void ModifiableChooser<T>::AddOption(std::string name, T object) { m_map[name] = object; }

template <typename T>
void ModifiableChooser<T>::RemoveOption(std::string name)
{
    if (m_map.contains(name))
    {
        if (m_defaultChoice == name)
        {
            m_defaultChoice = "";
        }

        if (m_selected == name)
        {
            m_selected = m_defaultChoice;
        }

        m_map.erase(name);
    }
}

template <typename T>
void ModifiableChooser<T>::ClearOptions()
{
    m_defaultChoice = "";
    m_selected = m_defaultChoice;

    m_map.clear();
}

template <typename T>
void ModifiableChooser<T>::SetOptions(std::map<std::string, T> options)
{
    ClearOptions();

    m_map = options;
}

template <typename T>
void ModifiableChooser<T>::SetDefaultOption(std::string name, T object)
{
    m_defaultChoice = name;
    AddOption(name, object);
}

template <typename T>
void ModifiableChooser<T>::SetOptions(std::map<std::string, T> options, std::string defaultName,
                                      T defaultObject)
{
    SetOptions(options);
    SetDefaultOption(defaultName, defaultObject);
    if (m_selected == "")
    {
        m_selected = defaultName;
    }
}

template <typename T>
T ModifiableChooser<T>::GetSelected()
{
    std::lock_guard<std::recursive_mutex> lk(m_mutex);

    try
    {
        // TODO
        if (m_selected != "")
        {
            return m_map[m_selected];
        }

        return m_map[m_defaultChoice];
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << '\n';
    }
}

template <typename T>
std::string ModifiableChooser<T>::GetSelectedKey()
{
    std::lock_guard<std::recursive_mutex> lk(m_mutex);

    try
    {
        // TODO
        if (m_selected != "")
        {
            return m_selected;
        }

        return m_defaultChoice;
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << '\n';
    }
}

template <typename T>
std::string ModifiableChooser<T>::GetNtSelected()
{
    std::optional<T> choice;
    std::function<void(std::optional<T>)> listener;
    std::string setSelectedTo;

    std::lock_guard<std::recursive_mutex> lk(m_mutex);

    try
    {
        if (m_selected != "")
        {
            setSelectedTo = m_selected;
        }
        else
        {
            setSelectedTo = m_defaultChoice;
        }

        if (setSelectedTo != m_previousValue && m_listener &&
            m_map.contains(setSelectedTo))
        {
            choice = m_map[setSelectedTo];
            listener = m_listener;
        }
        else
        {
            choice = std::nullopt;
            listener = nullptr;
        }

        m_previousValue = setSelectedTo;

        if (listener)
        {
            listener(choice);
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << '\n';
    }

    return setSelectedTo;
}

template <typename T>
void ModifiableChooser<T>::SetNtSelected(std::string val)
{
    std::optional<T> choice;
    std::function<void(std::optional<T>)> listener;

    std::lock_guard<std::recursive_mutex> lk(m_mutex);

    try
    {
        m_selected = val;

        if (!m_map.contains(m_selected))
        {
            m_selected = m_defaultChoice;
        }

        if (m_selected != m_previousValue && m_listener)
        {
            choice = m_map[val];
            listener = m_listener;
        }
        else
        {
            choice = std::nullopt;
            listener = nullptr;
        }

        m_previousValue = val;

        if (listener)
        {
            listener(choice);
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << '\n';
    }
}

template <typename T>
void ModifiableChooser<T>::OnChange(std::function<void(std::optional<T>)> listener)
{
    std::lock_guard<std::recursive_mutex> lk(m_mutex);
    m_listener = listener;
}

template <typename T>
void ModifiableChooser<T>::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("String Chooser");
    builder.PublishConstInteger(kInstance, m_instance);
    builder.AddStringProperty(
        kDefault, [this]
        { return m_defaultChoice; },
        nullptr);
    builder.AddStringArrayProperty(
        kOptions,
        [this]
        {
            auto keys = std::views::keys(m_map);
            return std::vector<std::string>{keys.begin(), keys.end()};
        },
        nullptr);
    builder.AddStringProperty(
        kActive, std::bind(&ModifiableChooser::GetSelectedKey, this), nullptr);
    builder.AddStringProperty(
        kSelected, std::bind(&ModifiableChooser::GetNtSelected, this),
        [this](std::string_view val)
        { SetNtSelected(std::string{val}); });
}