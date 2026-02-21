///////////////////////////////////////////////////////////
//                                                       //
//                        .                 .:-:         //
//                       :-:              :-::           //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                ---------------------                  //
//               .-------:. :---------:                  //
//              :-----:.     .-------.                   //
//             .:---:         .-----.                    //
//            .:-:.             :-:                      //
//          .-:.                 .                       //
//         .:                                            //
//                                                       //
//    ███╗   ██╗███████╗██╗  ██╗████████╗    ███████╗    //
//    ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝    ██╔════╝    //
//    ██╔██╗ ██║█████╗   ╚███╔╝    ██║       █████╗      //
//    ██║╚██╗██║██╔══╝   ██╔██╗    ██║       ██╔══╝      //
//    ██║ ╚████║███████╗██╔╝ ██╗   ██║       ███████╗    //
//    ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝       ╚══════╝    //
//                                                       //
///////////////////////////////////////////////////////////
//                                                       //
// Copyright (c) 2026 XAUT NEXT-E. All Rights Reserved.  //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Description: BY AI
// Thread-safe parameter server for vision modules.
// Supports dynamic parameter updates and retrievals with minimal latency.
// Wraps yaml-cpp with locks, exposing a YAML::Node-like interface.

#pragma once

#include <mutex>
#include <fstream>
#include <string>
#include <sstream>
#include <memory>

#include "yaml-cpp/yaml.h"
#include "ne_vision/utils/ne_log.hpp"

#define NV_PARAM NeParam::Instance()

namespace ne_vision
{

class NeParam
{
public:
  static NeParam& Instance()
  {
    static NeParam instance;
    return instance;
  }

  NeParam(const NeParam&) = delete;
  NeParam& operator=(const NeParam&) = delete;

  // Use recursive_mutex to prevent deadlocks when accessing NV_PARAM multiple
  // times in the same thread (e.g. nested calls or holding a node ref while
  // querying another).
  using LockType = std::unique_lock<std::recursive_mutex>;

  class NodeWrapper
  {
  public:
    NodeWrapper(std::shared_ptr<LockType> lock, YAML::Node node)
        : m_lock(std::move(lock)), m_node(node)
    {
    }

    // Access child by key
    template <typename Key>
    NodeWrapper operator[](const Key& key)
    {
      return NodeWrapper(m_lock, m_node[key]);
    }

    // Convert to type
    template <typename T>
    T as() const
    {
      return m_node.as<T>();
    }

    // Helper for default value (mimics fallback behavior)
    template <typename T>
    T as(const T& default_val) const
    {
      if (m_node.IsDefined() && !m_node.IsNull())
      {
        try
        {
          return m_node.as<T>();
        }
        catch (...)
        {
        }
      }
      return default_val;
    }

    // Assignment
    template <typename T>
    NodeWrapper& operator=(const T& rhs)
    {
      m_node = rhs;
      return *this;
    }

    // Iteration support
    YAML::iterator       begin() { return m_node.begin(); }
    YAML::iterator       end() { return m_node.end(); }
    YAML::const_iterator begin() const { return m_node.begin(); }
    YAML::const_iterator end() const { return m_node.end(); }

    // Common YAML::Node methods
    bool        IsDefined() const { return m_node.IsDefined(); }
    bool        IsNull() const { return m_node.IsNull(); }
    bool        IsSequence() const { return m_node.IsSequence(); }
    bool        IsMap() const { return m_node.IsMap(); }
    bool        IsScalar() const { return m_node.IsScalar(); }
    std::size_t size() const { return m_node.size(); }
    void push_back(const NodeWrapper& rhs) { m_node.push_back(rhs.m_node); }
    template <typename T>
    void push_back(const T& rhs)
    {
      m_node.push_back(rhs);
    }

  private:
    std::shared_ptr<LockType> m_lock;
    YAML::Node                m_node;
  };

  // Main access point
  template <typename Key>
  NodeWrapper operator[](const Key& key)
  {
    auto lock = std::make_shared<LockType>(m_mutex);
    return NodeWrapper(lock, m_node[key]);
  }

  bool LoadFromFile(const std::string& file_path)
  {
    std::unique_lock<std::recursive_mutex> lock(m_mutex);
    try
    {
      m_node = YAML::LoadFile(file_path);
      return true;
    }
    catch (const std::exception& e)
    {
      NV_ERROR("Failed to load param file: {} Error: {}", file_path, e.what());
      return false;
    }
  }

  bool Load(const std::string& file_path) { return LoadFromFile(file_path); }

  bool Save(const std::string& file_path)
  {
    std::unique_lock<std::recursive_mutex> lock(m_mutex);
    try
    {
      std::ofstream fout(file_path);
      fout << m_node;
      return true;
    }
    catch (const std::exception& e)
    {
      NV_ERROR("Failed to save param file: {} Error: {}", file_path, e.what());
      return false;
    }
  }

  std::string Dump()
  {
    std::unique_lock<std::recursive_mutex> lock(m_mutex);
    std::stringstream                      ss;
    ss << m_node;
    return ss.str();
  }

  void Reset()
  {
    std::unique_lock<std::recursive_mutex> lock(m_mutex);
    m_node = YAML::Node();
  }

private:
  NeParam() = default;
  mutable std::recursive_mutex m_mutex;
  YAML::Node                   m_node;
};

} // namespace ne_vision
