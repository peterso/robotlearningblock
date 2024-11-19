/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <esp_log.h>

#include <string>

/**
 * @struct SensorMeasurement
 *
 * @brief A union-based class for handling different types of sensor measurements
 */
struct SensorMeasurement
{
    /**
     * @struct Vector3
     * @brief Structure representing a 3D vector measurement
     */
    struct Vector3
    {
        float x;    ///< X component of the vector
        float y;    ///< Y component of the vector
        float z;    ///< Z component of the vector
    };

    // Type aliases for measurement types
    using BooleanType = bool;         ///< Type alias for boolean measurements
    using AnalogType = float;         ///< Type alias for analog (float) measurements
    using Vector3Type = Vector3;      ///< Type alias for 3D vector measurements

    /**
     * @enum Type
     * @brief Enumeration of supported measurement types
     */
    enum class Type
    {
        BOOLEAN,    ///< Boolean measurement type
        ANALOG,     ///< Analog (float) measurement type
        VECTOR3     ///< 3D vector measurement type
    };

    /**
     * @brief Constructs a new boolean SensorMeasurement
     *
     * @param boolean_value Boolean value to store
     */
    SensorMeasurement(
            const BooleanType& boolean_value)
        : type_(Type::BOOLEAN)
        , boolean_value(boolean_value)
    {
    }

    /**
     * @brief Constructs a new analog SensorMeasurement
     *
     * @param analog_value Float value to store
     */
    SensorMeasurement(
            const AnalogType& analog_value)
        : type_(Type::ANALOG)
        , analog_value(analog_value)
    {
    }

    /**
     * @brief Constructs a new vector SensorMeasurement
     *
     * @param vector3 3D vector value to store
     */
    SensorMeasurement(
            const Vector3Type& vector3)
        : type_(Type::VECTOR3)
        , vector3(vector3)
    {
    }

    /**
     * @brief Gets the type of measurement stored
     * @return Type enum indicating measurement type
     */
    Type get_type() const
    {
        return type_;
    }

    /**
     * @brief Converts the measurement to a string representation
     * @return String representing the stored value
     */
    std::string to_string() const
    {
        switch (type_)
        {
            case Type::BOOLEAN:
                return boolean_value ? "true" : "false";
                break;
            case Type::ANALOG:
                return std::to_string(analog_value);
                break;
            case Type::VECTOR3:
                return "[" + std::to_string(vector3.x) + ", " + std::to_string(vector3.y) + ", " + std::to_string(
                    vector3.z) + "]";
                break;
        }

        return "unknown";
    }

    /**
     * @brief Compares two measurements for equality within tolerance
     *
     * @param a First measurement to compare
     * @param b Second measurement to compare
     * @param tolerance Maximum allowed difference for floating point comparisons
     *
     * @return true if measurements are equal within tolerance, false otherwise
     */
    static bool equal(
            const SensorMeasurement& a,
            const SensorMeasurement& b,
            const float tolerance = 0.00)
    {
        bool ret = false;

        if (a.type_ == b.type_)
        {
            switch (a.type_)
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

    /**
     * @brief Compares if first measurement is greater than or equal to second
     *
     * @param a First measurement to compare
     * @param b Second measurement to compare
     *
     * @return true if a >= b component-wise, false otherwise
     */
    static bool greater_or_equal(
            const SensorMeasurement& a,
            const SensorMeasurement& b)
    {
        bool ret = false;

        if (a.type_ == b.type_)
        {
            switch (a.type_)
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

    /**
     * @brief Equality comparison operator with boolean values
     *
     * @param b Boolean value to compare against
     *
     * @return true if measurement is boolean and equals b, false otherwise
     */
    bool operator ==(
            const bool& b) const
    {
        return type_ == Type::BOOLEAN && boolean_value == b;
    }

    /**
     * @brief Gets the stored boolean value
     *
     * @return Reference to the stored boolean value
     */
    const BooleanType& get_boolean() const
    {
        return boolean_value;
    }

    /**
     * @brief Gets the stored analog value
     *
     * @return Reference to the stored analog value
     */
    const AnalogType& get_analog() const
    {
        return analog_value;
    }

    /**
     * @brief Gets the stored vector value
     *
     * @return Reference to the stored vector value
     */
    const Vector3Type& get_vector3() const
    {
        return vector3;
    }

private:

    /**
     * @brief Compares two float values within tolerance
     * @param a First float to compare
     * @param b Second float to compare
     * @param tolerance Maximum allowed difference
     * @return true if |a-b| < tolerance, false otherwise
     */
    static bool compare_floats(
            float a,
            float b,
            float tolerance)
    {
        return std::abs(a - b) < tolerance;
    }

    Type type_;    ///< Type of measurement currently stored

    /**
     * @brief Union storing the actual measurement value
     */
    union
    {
        BooleanType boolean_value;    ///< Boolean measurement storage
        AnalogType analog_value;      ///< Analog measurement storage
        Vector3Type vector3;          ///< Vector measurement storage
    };
};
