#include <iostream>
#include <pybind11/embed.h> // Embed Python
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <fstream>


#include <boost/archive/text_oarchive.hpp>
#include <boost/variant.hpp>
#include <boost/serialization/vector.hpp>

namespace py = pybind11;

class GroundTruth {
public:
    GroundTruth() = default;
    GroundTruth(const GroundTruth &other) :
        acceleration(other.acceleration), velocity(other.velocity), position(other.position),
        angularAcceleration(other.angularAcceleration), angularVelocity(other.angularVelocity),
        distance(other.distance) {}
    GroundTruth &operator=(const GroundTruth &other) {
        if (this == &other)
            return *this;
        acceleration = other.acceleration;
        velocity = other.velocity;
        position = other.position;
        angularAcceleration = other.angularAcceleration;
        angularVelocity = other.angularVelocity;
        distance = other.distance;
        return *this;
    }

    std::vector<std::vector<double>> &acceleration1() { return acceleration; }

    void set_acceleration(std::vector<std::vector<double>> &&acceleration) {
        this->acceleration = std::move(acceleration);
    }
    void set_velocity(std::vector<std::vector<double>> &&velocity) { this->velocity = std::move(velocity); }
    void set_position(std::vector<std::vector<double>> &&position) { this->position = std::move(position); }
    void set_angular_acceleration(std::vector<std::vector<double>> &&angular_acceleration) {
        angularAcceleration = std::move(angular_acceleration);
    }
    void set_angular_velocity(std::vector<std::vector<double>> &&angular_velocity) {
        angularVelocity = std::move(angular_velocity);
    }
    void set_distance(std::vector<std::vector<double>> &&distance) { this->distance = std::move(distance); }

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & acceleration;
        ar & velocity;
        ar & position;

        ar & angularAcceleration;
        ar & angularVelocity;
        ar & distance;
    }

    std::vector<std::vector<double>> acceleration, velocity, position;
    std::vector<std::vector<double>> angularAcceleration, angularVelocity, distance;
};

class SensorData {
public:
    SensorData() = default;
    SensorData(std::vector<std::vector<double>> &&data, std::vector<double> &&timestamp) :
        data(std::move(data)), timestamp(std::move(timestamp)) {}

    void set_data(std::vector<std::vector<double>> &&data) { this->data = std::move(data); }
    void set_timestamp(std::vector<double> &&timestamp) { this->timestamp = std::move(timestamp); }
    std::vector<std::vector<double>> &data1() { return data; }
    std::vector<double> &timestamp1() { return timestamp; }

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & data;
        ar & timestamp;
    }

    std::vector<std::vector<double>> data;
    std::vector<double> timestamp;
};

class IMUMeasurement {
public:
    IMUMeasurement() = default;
    IMUMeasurement(const SensorData &acceleration, const SensorData &angular_velocity) :
        acceleration(acceleration), angularVelocity(angular_velocity) {}

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & acceleration;
        ar & angularVelocity;
    }

    SensorData acceleration, angularVelocity;
};

class Data {
public:
    Data() = default;
    Data(const GroundTruth &ground_truth, const IMUMeasurement &imu_measurements, const SensorData &gnss_measurement,
         const SensorData &li_dar_measurement) :
        groundTruth(ground_truth), IMUMeasurements(imu_measurements), GNSSMeasurement(gnss_measurement),
        LiDARMeasurement(li_dar_measurement) {}
    Data(Data &&other) :
        groundTruth(std::move(other.groundTruth)), IMUMeasurements(std::move(other.IMUMeasurements)),
        GNSSMeasurement(std::move(other.GNSSMeasurement)), LiDARMeasurement(std::move(other.LiDARMeasurement)) {}
    Data &operator=(Data &&other) {
        if (this == &other)
            return *this;
        groundTruth = std::move(other.groundTruth);
        IMUMeasurements = std::move(other.IMUMeasurements);
        GNSSMeasurement = std::move(other.GNSSMeasurement);
        LiDARMeasurement = std::move(other.LiDARMeasurement);
        return *this;
    }
    GroundTruth &ground_truth() { return groundTruth; }
    IMUMeasurement &imu_measurements() { return IMUMeasurements; }
    SensorData &gnss_measurement() { return GNSSMeasurement; }
    SensorData &li_dar_measurement() { return LiDARMeasurement; }

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & groundTruth;
        ar & IMUMeasurements;
        ar & GNSSMeasurement;
        ar & LiDARMeasurement;
    }

    GroundTruth groundTruth;
    IMUMeasurement IMUMeasurements;
    SensorData GNSSMeasurement, LiDARMeasurement;
};

class foo : public boost::static_visitor<> {
public:
    void operator()(GroundTruth &x) const {



        py::object builtins = py::module_::import("builtins");
        py::object pickle = py::module_::import("pickle");
        py::object f = builtins.attr("open")("data/pt1_data.pkl", "rb");
        py::dict dictionary = pickle.attr("load")(f).cast<py::dict>();
        f.attr("close")();

        py::object obj = dictionary["gt"];
        py::array dataArr = obj.attr("a").cast<py::array>();
        const auto acceleration = dataArr.unchecked<double, 2>();

        std::vector<std::vector<double>> temp;
        for (auto i = 0; i < acceleration.shape(0); ++i) {
            temp.push_back(std::vector<double>{acceleration(i, 0), acceleration(i, 1), acceleration(i, 2)});
        }

        x.set_acceleration(std::move(temp));

        dataArr = obj.attr("v").cast<py::array>();
        const auto velocity = dataArr.unchecked<double, 2>();
        for (auto i = 0; i < velocity.shape(0); ++i) {
            temp.push_back(std::vector<double>{velocity(i, 0), velocity(i, 1), velocity(i, 2)});
        }

        x.set_velocity(std::move(temp));

        dataArr = obj.attr("p").cast<py::array>();
        const auto position = dataArr.unchecked<double, 2>();
        for (auto i = 0; i < position.shape(0); ++i) {
            temp.push_back(std::vector<double>{position(i, 0), position(i, 1), position(i, 2)});
        }

        x.set_position(std::move(temp));

        dataArr = obj.attr("alpha").cast<py::array>();
        const auto angularAcceleration = dataArr.unchecked<double, 2>();
        for (auto i = 0; i < angularAcceleration.shape(0); ++i) {
            temp.push_back(std::vector<double>{angularAcceleration(i, 0), angularAcceleration(i, 1), angularAcceleration(i, 2)});
        }

        x.set_angular_acceleration(std::move(temp));

        dataArr = obj.attr("w").cast<py::array>();
        const auto angularVelocity = dataArr.unchecked<double, 2>();
        for (auto i = 0; i < angularVelocity.shape(0); ++i) {
            temp.push_back(std::vector<double>{angularVelocity(i, 0), angularVelocity(i, 1), angularVelocity(i, 2)});
        }

        x.set_angular_velocity(std::move(temp));

        dataArr = obj.attr("r").cast<py::array>();
        const auto distance = dataArr.unchecked<double, 2>();
        for (auto i = 0; i < distance.shape(0); ++i) {
            temp.push_back(std::vector<double>{distance(i, 0), distance(i, 1), distance(i, 2)});
        }

        x.set_distance(std::move(temp));


    }

    void operator()(std::unordered_map<std::string, SensorData> &x) const {


        py::object builtins = py::module_::import("builtins");
        py::object pickle = py::module_::import("pickle");
        py::object f = builtins.attr("open")("data/pt1_data.pkl", "rb");
        py::dict dictionary = pickle.attr("load")(f).cast<py::dict>();
        f.attr("close")();

        py::object obj = dictionary["imu_f"];
        py::array dataArr = obj.attr("data").cast<py::array>();
        const auto acceleration = dataArr.unchecked<double, 2>();

        std::vector<std::vector<double>> temp;
        for (auto i = 0; i < acceleration.shape(0); ++i) {
            temp.push_back(std::vector<double>{acceleration(i, 0), acceleration(i, 1), acceleration(i, 2)});
        }

        std::vector<double> temp_timestamp;

        dataArr = obj.attr("t").cast<py::array>();
        const auto timestamp = dataArr.unchecked<double, 1>();
        for (auto i = 0; i < timestamp.shape(0); ++i) {
            temp_timestamp.push_back(timestamp(i));
        }

        x["Acceleration"] = SensorData {std::move(temp), std::move(temp_timestamp)};

        py::object obj1 = dictionary["imu_w"];
        dataArr = obj1.attr("data").cast<py::array>();
        const auto angularVelocity = dataArr.unchecked<double, 2>();

        for (auto i = 0; i < angularVelocity.shape(0); ++i) {
            temp.push_back(std::vector<double>{angularVelocity(i, 0), angularVelocity(i, 1), angularVelocity(i, 2)});
        }

        dataArr = obj1.attr("t").cast<py::array>();
        const auto timestamp1 = dataArr.unchecked<double, 1>();
        for (auto i = 0; i < timestamp1.shape(0); ++i) {
            temp_timestamp.push_back(timestamp1(i));
        }

        x["angularVelocity"] = SensorData {std::move(temp), std::move(temp_timestamp)};

        py::object obj2 = dictionary["gnss"];
        dataArr = obj2.attr("data").cast<py::array>();
        const auto gnss = dataArr.unchecked<double, 2>();

        for (auto i = 0; i < gnss.shape(0); ++i) {
            temp.push_back(std::vector<double>{gnss(i, 0), gnss(i, 1), gnss(i, 2)});
        }

        dataArr = obj2.attr("t").cast<py::array>();
        const auto timestamp2 = dataArr.unchecked<double, 1>();
        for (auto i = 0; i < timestamp2.shape(0); ++i) {
            temp_timestamp.push_back(timestamp2(i));
        }

        x["GNSS"] = SensorData {std::move(temp), std::move(temp_timestamp)};

        py::object obj3 = dictionary["lidar"];
        dataArr = obj3.attr("data").cast<py::array>();
        const auto lidar = dataArr.unchecked<double, 2>();

        for (auto i = 0; i < lidar.shape(0); ++i) {
            temp.push_back(std::vector<double>{lidar(i, 0), lidar(i, 1), lidar(i, 2)});
        }

        dataArr = obj3.attr("t").cast<py::array>();
        const auto timestamp3 = dataArr.unchecked<double, 1>();
        for (auto i = 0; i < timestamp3.shape(0); ++i) {
            temp_timestamp.push_back(timestamp3(i));
        }

        x["LiDAR"] = SensorData {std::move(temp), std::move(temp_timestamp)};


    }
};

class Loader {
public:
    Loader() = default;

    void load(boost::variant<GroundTruth, std::unordered_map<std::string, SensorData>> &data) const {
        boost::apply_visitor(foo(), data);
    }
};

/*

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
        auto r = array.unchecked<double, 2>();
        builtins.attr("print")(r(0, 1));

        py::exec(
            "import numpy\n",
            mainScope);
    }
    catch (py::error_already_set const &pythonErr) {  std::cout << pythonErr.what(); }
}
*/


int main() {
    py::initialize_interpreter();
    boost::variant<GroundTruth, std::unordered_map<std::string, SensorData>> groundTruth = GroundTruth();
    boost::variant<GroundTruth, std::unordered_map<std::string, SensorData>> sensorMap = std::unordered_map<std::string, SensorData>();


    Loader loader {};

    loader.load(groundTruth);
    loader.load(sensorMap);

    GroundTruth gt;

    if (GroundTruth* a = boost::get<GroundTruth>(&groundTruth)) {
        gt = *a;

        // std::cout << a->acceleration1()[1][2] << std::endl;
    }

    Data myData;
    std::ofstream ofs("C:/Users/gediz/OneDrive/Desktop/mydata");

    // std::cout << gt.acceleration1()[0][0];

    if (auto* umap = boost::get<std::unordered_map<std::string, SensorData>>(&sensorMap)) {
        // std::cout << (*umap)["LiDAR"].data1()[0][0] << std::endl;

        IMUMeasurement measurement((*umap)["Acceleration"], (*umap)["angularVelocity"]);
        Data dd(gt, measurement, (*umap)["GNSS"], (*umap)["LiDAR"]);
        myData = std::move(dd);

    }


    // std::cout << myData.gnss_measurement().data1()[0][0] << std::endl;
    {
        boost::archive::text_oarchive oa(ofs);
        // write class instance to archive
        oa << myData;
        // archive and stream closed when destructors are called
    }


    py::finalize_interpreter();
}
