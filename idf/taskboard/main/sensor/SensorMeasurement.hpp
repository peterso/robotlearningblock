/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <esp_log.h>

#include <string>

struct SensorMeasurement
{
    // Allowed types
    struct Vector3
    {
        float x;
        float y;
        float z;
    };

    using Vector3Type = Vector3;
    using AnalogType = float;
    using BooleanType = bool;

    enum class Type
    {
        BOOLEAN,
        ANALOG,
        VECTOR3
    };

    SensorMeasurement(const BooleanType& boolean_value)
    : type_(Type::BOOLEAN),
      boolean_value(boolean_value)
    {}

    SensorMeasurement(const AnalogType& analog_value)
    : type_(Type::ANALOG),
      analog_value(analog_value)
    {}

    SensorMeasurement(const Vector3Type& vector3)
    : type_(Type::VECTOR3),
      vector3(vector3)
    {}

    Type get_type() const
    {
        return type_;
    }

    std::string to_string() const
    {
        switch(type_)
        {
            case Type::BOOLEAN:
                return boolean_value ? "true" : "false";
                break;
            case Type::ANALOG:
                return std::to_string(analog_value);
                break;
            case Type::VECTOR3:
                return "[" + std::to_string(vector3.x) + ", " + std::to_string(vector3.y) + ", " + std::to_string(vector3.z) + "]";
                break;
        }

        return "unknown";
    }

    // Comparators
    static bool equal(const SensorMeasurement & a, const SensorMeasurement & b, const float tolerance = 0.00)
    {
        bool ret = false;

        if (a.type_ == b.type_)
        {
            switch(a.type_)
            {
                case Type::BOOLEAN:
                    ret = a.boolean_value == b.boolean_value;
                    break;
                case Type::ANALOG:
                    ret = compare_floats(a.analog_value, b.analog_value, tolerance);
                    break;
                case Type::VECTOR3:
                    ret = compare_floats(a.vector3.x, b.vector3.x, tolerance) &&
                          compare_floats(a.vector3.y, b.vector3.y, tolerance) &&
                          compare_floats(a.vector3.z, b.vector3.z, tolerance);
                    break;
            }
        }

        return ret;
    }

    static bool greater_or_equal(const SensorMeasurement & a, const SensorMeasurement & b)
    {
        bool ret = false;

        if (a.type_ == b.type_)
        {
            switch(a.type_)
            {
                case Type::BOOLEAN:
                    ret = a.boolean_value == b.boolean_value;
                    break;
                case Type::ANALOG:
                    ret = a.analog_value >= b.analog_value;
                    break;
                case Type::VECTOR3:
                    ret = a.vector3.x >= b.vector3.x &&
                          a.vector3.y >= b.vector3.y &&
                          a.vector3.z >= b.vector3.z;
                    break;
            }
        }

        return ret;
    }

    // Operator equal to compare with boolean
    bool operator==(const bool & b) const
    {
        return type_ == Type::BOOLEAN && boolean_value == b;
    }

    // Getters
    const BooleanType& get_boolean() const
    {
        return boolean_value;
    }

    const AnalogType& get_analog() const
    {
        return analog_value;
    }

    const Vector3Type& get_vector3() const
    {
        return vector3;
    }

private:

    Type type_;

    union
    {
        BooleanType boolean_value;
        AnalogType analog_value;
        Vector3Type vector3;
    };

    static bool compare_floats(float a, float b, float tolerance)
    {
        return std::abs(a - b) < tolerance;
    }
};