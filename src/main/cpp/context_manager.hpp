/**
 * Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 *
 * @file context_manager.hpp
 * @author Gautier Bureau <gautier.bureau at rte-france.com>
 */

#ifndef CONTEXT_MANAGER_HPP
#define CONTEXT_MANAGER_HPP

#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace powsybl {

/**
 * Thread-safe registry of solver contexts keyed by a string id. Each solver
 * backend (sparse LU, Gauss-Newton CHOLMOD) owns one global instance. Contexts
 * are heap-allocated and owned by the manager; create/find hand back references
 * that stay valid until removeContext().
 *
 * Note the registry - not the context - is what this mutex protects: concurrent
 * use of distinct ids is safe, concurrent use of the same id is not.
 *
 * `label` is prepended to error messages so they keep their per-backend
 * wording, e.g. label "CHOLMOD " yields "CHOLMOD Context <id> not found".
 */
template <class Ctx>
class ContextManager {
public:
    explicit ContextManager(std::string label = "") : _label(std::move(label)) {}

    ContextManager(const ContextManager&) = delete;
    ContextManager& operator=(const ContextManager&) = delete;
    ~ContextManager() = default;

    template <class... Args>
    Ctx& createContext(const std::string& id, Args&&... args) {
        std::lock_guard<std::mutex> lk(_mutex);
        if (_contexts.find(id) != _contexts.end()) {
            throw std::runtime_error(_label + "Context " + id + " already exists");
        }
        std::unique_ptr<Ctx> context(new Ctx(std::forward<Args>(args)...));
        auto it = _contexts.insert(std::make_pair(id, std::move(context)));
        return *it.first->second;
    }

    Ctx& findContext(const std::string& id) {
        std::lock_guard<std::mutex> lk(_mutex);
        auto it = _contexts.find(id);
        if (it == _contexts.end()) {
            throw std::runtime_error(_label + "Context " + id + " not found");
        }
        return *it->second;
    }

    void removeContext(const std::string& id) {
        std::lock_guard<std::mutex> lk(_mutex);
        _contexts.erase(id);
    }

private:
    std::string _label;
    std::map<std::string, std::unique_ptr<Ctx>> _contexts;
    std::mutex _mutex;
};

}  // namespace powsybl

#endif // CONTEXT_MANAGER_HPP
