#include <cmath>
#include <stdexcept>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

py::array_t<double> compute_cost_matrix(
    py::array_t<double, py::array::c_style | py::array::forcecast> drone_positions,
    py::array_t<double, py::array::c_style | py::array::forcecast> target_positions,
    py::array_t<double, py::array::c_style | py::array::forcecast> target_uncertainties,
    double gamma = 0.01,
    bool uncertainty_priority = true,
    double uncertainty_scale_m = 100.0
) {
    py::buffer_info drone_buf = drone_positions.request();
    py::buffer_info target_buf = target_positions.request();
    py::buffer_info uncertainty_buf = target_uncertainties.request();

    if (drone_buf.ndim != 2 || drone_buf.shape[1] < 3) {
        throw std::invalid_argument("drone_positions must have shape (num_drones, 3)");
    }
    if (target_buf.ndim != 2 || target_buf.shape[1] < 3) {
        throw std::invalid_argument("target_positions must have shape (num_targets, 3)");
    }
    if (uncertainty_buf.ndim != 1 || uncertainty_buf.shape[0] != target_buf.shape[0]) {
        throw std::invalid_argument("target_uncertainties length must match target_positions rows");
    }

    const py::ssize_t num_drones = drone_buf.shape[0];
    const py::ssize_t num_targets = target_buf.shape[0];

    const auto* drones = static_cast<const double*>(drone_buf.ptr);
    const auto* targets = static_cast<const double*>(target_buf.ptr);
    const auto* uncertainties = static_cast<const double*>(uncertainty_buf.ptr);

    const py::ssize_t drone_stride0 = drone_buf.strides[0] / static_cast<py::ssize_t>(sizeof(double));
    const py::ssize_t drone_stride1 = drone_buf.strides[1] / static_cast<py::ssize_t>(sizeof(double));
    const py::ssize_t target_stride0 = target_buf.strides[0] / static_cast<py::ssize_t>(sizeof(double));
    const py::ssize_t target_stride1 = target_buf.strides[1] / static_cast<py::ssize_t>(sizeof(double));
    const py::ssize_t uncertainty_stride0 = uncertainty_buf.strides[0] / static_cast<py::ssize_t>(sizeof(double));

    py::array_t<double> cost_matrix({num_drones, num_targets});
    py::buffer_info cost_buf = cost_matrix.request();
    auto* cost = static_cast<double*>(cost_buf.ptr);
    const py::ssize_t cost_stride0 = cost_buf.strides[0] / static_cast<py::ssize_t>(sizeof(double));
    const py::ssize_t cost_stride1 = cost_buf.strides[1] / static_cast<py::ssize_t>(sizeof(double));

    double max_uncertainty = 0.0;
    {
        py::gil_scoped_release release;

        for (py::ssize_t j = 0; j < num_targets; ++j) {
            const double uncertainty = uncertainties[j * uncertainty_stride0];
            if (uncertainty > max_uncertainty) {
                max_uncertainty = uncertainty;
            }
        }

        for (py::ssize_t i = 0; i < num_drones; ++i) {
            const double drone_x = drones[i * drone_stride0 + 0 * drone_stride1];
            const double drone_y = drones[i * drone_stride0 + 1 * drone_stride1];
            const double drone_z = drones[i * drone_stride0 + 2 * drone_stride1];

            for (py::ssize_t j = 0; j < num_targets; ++j) {
                const double target_x = targets[j * target_stride0 + 0 * target_stride1];
                const double target_y = targets[j * target_stride0 + 1 * target_stride1];
                const double target_z = targets[j * target_stride0 + 2 * target_stride1];

                const double dx = target_x - drone_x;
                const double dy = target_y - drone_y;
                const double dz = target_z - drone_z;
                const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                const double uncertainty = uncertainties[j * uncertainty_stride0];
                double uncertainty_term = gamma * uncertainty;
                if (uncertainty_priority && max_uncertainty > 0.0) {
                    uncertainty_term = -gamma * (uncertainty / max_uncertainty) * uncertainty_scale_m;
                }

                cost[i * cost_stride0 + j * cost_stride1] = distance + uncertainty_term;
            }
        }
    }

    return cost_matrix;
}

PYBIND11_MODULE(fast_math, m) {
    m.doc() = "C++ acceleration helpers for RADAR-SWARM math kernels";
    m.def(
        "compute_cost_matrix",
        &compute_cost_matrix,
        py::arg("drone_positions"),
        py::arg("target_positions"),
        py::arg("target_uncertainties"),
        py::arg("gamma") = 0.01,
        py::arg("uncertainty_priority") = true,
        py::arg("uncertainty_scale_m") = 100.0,
        R"pbdoc(
            Compute the drone-target assignment cost matrix.

            C_ij = ||p_i - x_j||_2 - gamma * s * trace(P_j) / max_k(trace(P_k))

            When uncertainty_priority is false:

            C_ij = ||p_i - x_j||_2 + gamma * trace(P_j)
        )pbdoc"
    );
}
