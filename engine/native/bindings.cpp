#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "GR.h"
#include "lutio.hpp"
#include "optimizer.hpp"

namespace py = pybind11;

namespace {

template <typename T>
T config_value(const py::dict &config, const char *key, T fallback) {
    py::str py_key(key);
    return config.contains(py_key) ? config[py_key].cast<T>() : fallback;
}

std::array<double, 2> double_point(const py::handle &value) {
    auto point = py::reinterpret_borrow<py::sequence>(value);
    if (py::len(point) != 2) {
        throw std::invalid_argument("A point must contain exactly two coordinates");
    }
    return {point[0].cast<double>(), point[1].cast<double>()};
}

AStar::Vec2i grid_point(const py::handle &value) {
    auto point = double_point(value);
    return {static_cast<int>(point[0]), static_cast<int>(point[1])};
}

AStar::CoordinateList coordinate_list(const py::handle &value) {
    AStar::CoordinateList result;
    for (const auto &point : py::reinterpret_borrow<py::sequence>(value)) {
        result.push_back(grid_point(point));
    }
    return result;
}

py::dict route(const py::list &net_values, int width_grid, int height_grid,
               const py::list &obstacle_values, const py::dict &config,
               const py::list &seed_path_values,
               const py::list &history_cost_values) {
    if (width_grid <= 0 || height_grid <= 0) {
        throw std::invalid_argument("width_grid and height_grid must be positive");
    }

    const double pitch = config_value<double>(config, "pitch", 10.0);
    std::vector<Net> nets;
    nets.reserve(py::len(net_values));
    for (size_t index = 0; index < py::len(net_values); ++index) {
        py::dict value = net_values[index].cast<py::dict>();
        auto source = double_point(value["source"]);
        auto target = double_point(value["target"]);
        std::string name = value.contains("name")
                               ? value["name"].cast<std::string>()
                               : std::to_string(index);
        Net net(name, Point(source[0] * pitch, source[1] * pitch),
                Point(target[0] * pitch, target[1] * pitch));
        net.setSourceDirection(value.contains("source_direction")
                                   ? value["source_direction"].cast<int>()
                                   : -1);
        net.setTargetDirection(value.contains("target_direction")
                                   ? value["target_direction"].cast<int>()
                                   : -1);
        nets.push_back(std::move(net));
    }
    if (nets.empty()) {
        throw std::invalid_argument("At least one net is required");
    }

    std::vector<AStar::Vec2i> obstacles;
    obstacles.reserve(py::len(obstacle_values));
    for (const auto &value : obstacle_values) obstacles.push_back(grid_point(value));

    std::vector<AStar::CoordinateList> seed_paths;
    seed_paths.reserve(py::len(seed_path_values));
    for (const auto &value : seed_path_values) seed_paths.push_back(coordinate_list(value));

    std::vector<std::pair<AStar::Vec2i, double>> history_costs;
    history_costs.reserve(py::len(history_cost_values));
    for (const auto &value : history_cost_values) {
        auto item = py::reinterpret_borrow<py::sequence>(value);
        if (py::len(item) != 3) {
            throw std::invalid_argument("A history cost must be [x, y, cost]");
        }
        history_costs.push_back({{item[0].cast<int>(), item[1].cast<int>()},
                                 item[2].cast<double>()});
    }

    const bool diagonal = config_value<bool>(config, "diagonal", false);
    const int direction = config_value<int>(config, "direction", 1);
    const bool use_input_order =
        config_value<std::string>(config, "order", "input") == "input";
    const bool block_pins = config_value<bool>(config, "block_pins", true);
    const int target_guard = config_value<int>(config, "target_guard", 0);
    const double rudy_weight = config_value<double>(config, "rudy_weight", 0.0);
    const bool block_routed_paths =
        config_value<bool>(config, "block_routed_paths", false);
    const bool reserve_pin_stubs =
        config_value<bool>(config, "reserve_pin_stubs", false);
    const bool congestion_aware =
        config_value<bool>(config, "congestion_aware", false);
    const double congestion_beta =
        config_value<double>(config, "congestion_beta", 1.5);
    const double congestion_t =
        config_value<double>(config, "congestion_t", 1.6094379124341003);
    const double edge_capacity = config_value<double>(config, "edge_capacity", 1.0);
    const double overflow_penalty =
        config_value<double>(config, "overflow_penalty", 99999.0);
    const double min_bend_radius_grid =
        config_value<double>(config, "min_bend_radius_grid", 0.0);
    const double history_penalty =
        config_value<double>(config, "history_penalty", 0.0);
    const size_t hard_seed_count = config_value<size_t>(config, "hard_seed_count", 0);
    const double path_loss = config_value<double>(config, "path_loss", PATH_LOSS);
    const double crossing_loss =
        config_value<double>(config, "crossing_loss", CROSS_LOSS);
    const double bend_loss_30 =
        config_value<double>(config, "bend_loss_30", BEND_LOSS_30);
    const double bend_loss_45 =
        config_value<double>(config, "bend_loss_45", BEND_LOSS_45);
    const double bend_loss_60 =
        config_value<double>(config, "bend_loss_60", BEND_LOSS_60);
    const double bend_loss_90 =
        config_value<double>(config, "bend_loss_90", BEND_LOSS_90);

    std::unique_ptr<GlobalRouting> routing;
    {
        py::gil_scoped_release release;
        routing = std::make_unique<GlobalRouting>(
            width_grid * pitch, height_grid * pitch, std::move(nets),
            AStar::Heuristic::octagonal, diagonal, direction, use_input_order,
            block_pins, target_guard, std::move(obstacles), rudy_weight,
            block_routed_paths, reserve_pin_stubs, std::move(seed_paths),
            congestion_aware, congestion_beta, congestion_t, edge_capacity,
            overflow_penalty, min_bend_radius_grid, std::move(history_costs),
            history_penalty, hard_seed_count, path_loss, crossing_loss,
            bend_loss_30, bend_loss_45, bend_loss_60, bend_loss_90);
    }

    std::vector<std::vector<std::array<int, 2>>> routes;
    std::vector<double> costs;
    for (const auto &routed : routing->getRoutedNets()) {
        std::vector<std::array<int, 2>> path;
        path.reserve(routed.path.size());
        for (const auto &point : routed.path) path.push_back({point.x, point.y});
        routes.push_back(std::move(path));
        costs.push_back(routed.cost);
    }
    py::dict result;
    result["routes"] = std::move(routes);
    result["route_costs"] = std::move(costs);
    result["total_cost"] = routing->getTotalCost();
    return result;
}

class NativeLutOptimizer {
public:
    NativeLutOptimizer(const std::string &path, int max_level)
        : max_level_(max_level) {
        py::gil_scoped_release release;
        if (!lutio::load_binary(lut_, path)) {
            throw std::runtime_error("Failed to load LUT: " + path);
        }
    }

    std::vector<std::vector<std::array<int, 2>>>
    optimize(const std::vector<std::vector<std::array<int, 2>>> &values) const {
        if (values.empty()) return {};
        std::vector<pathio::Path> paths;
        paths.reserve(values.size());
        for (const auto &value : values) {
            pathio::Path path;
            for (const auto &point : value) path.add_point(point[0], point[1]);
            paths.push_back(std::move(path));
        }
        {
            py::gil_scoped_release release;
            pathio::sort_paths_by_bends(paths);
            Optimizer optimizer(lut_, max_level_);
            optimizer.optimize_paths(paths);
        }
        std::vector<std::vector<std::array<int, 2>>> result;
        result.reserve(paths.size());
        for (const auto &path : paths) {
            std::vector<std::array<int, 2>> points;
            points.reserve(path.points.size());
            for (const auto &point : path.points) points.push_back({point.x, point.y});
            result.push_back(std::move(points));
        }
        return result;
    }

private:
    LUT lut_;
    int max_level_;
};

} // namespace

PYBIND11_MODULE(_elite_cpp, module) {
    module.doc() = "In-process C++ routing and LUT kernels for ELITE";
    module.def("route", &route, py::arg("nets"), py::arg("width_grid"),
               py::arg("height_grid"), py::arg("obstacles") = py::list(),
               py::arg("config") = py::dict(), py::arg("seed_paths") = py::list(),
               py::arg("history_costs") = py::list());
    py::class_<NativeLutOptimizer>(module, "LutOptimizer")
        .def(py::init<const std::string &, int>(), py::arg("lut_path"),
             py::arg("max_level") = 10)
        .def("optimize", &NativeLutOptimizer::optimize, py::arg("paths"));
}
