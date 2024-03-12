#include <pybind11/pybind11.h>
#include <pybind11/embed.h> // Embed Python
#include <pybind11/numpy.h>
#include <iostream>

namespace py = pybind11;
void test() {
    try {
        py::object mainScope = py::module::import("__main__").attr("__dict__");
        py::object np = py::module_::import("numpy"); // Import the numpy module
        py::object pickle = py::module_::import("pickle");
        // py::object os = py::module_::import("os");
        py::object builtins = py::module_::import("builtins");
        // py::object arr = os.attr("getcwd")();
        py::object file = py::module_::import("builtins").attr("open")("data/pt1_data.pkl", "rb");
        py::dict obj = pickle.attr("load")(file).cast<py::dict>();
        file.attr("close")();

        py::object value = obj["lidar"];
        py::array array = value.attr("data").cast<py::array>();
        builtins.attr("print")(array.shape());

        py::exec(
            "import numpy\n",
            mainScope);
    }
    catch (py::error_already_set const &pythonErr) {  std::cout << pythonErr.what(); }
}
int main() {
    py::initialize_interpreter();
    test();
    test();
    py::finalize_interpreter();
}
