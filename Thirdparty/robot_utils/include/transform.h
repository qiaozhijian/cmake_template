#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

class Transform {
public:
    inline Transform()
            : pos_(&parameters_[0]), rot_(&parameters_[3]), dcm_(Eigen::Matrix3d::Identity()) {
        pos_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        rot_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    }

    inline Transform(Eigen::Matrix4d const &mat)
            : pos_(&parameters_[0]), rot_(&parameters_[3]), dcm_(Eigen::Matrix3d::Identity()) {
        pos_ = mat.block<3, 1>(0, 3);
        rot_ = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
    }

    inline virtual ~Transform() = default;

    inline Transform(const Transform &other)
            : parameters_(other.parameters_), pos_(&parameters_[0]), rot_(&parameters_[3]), dcm_(other.dcm()) {}

    inline Eigen::Map<Eigen::Vector3d> const &p() const noexcept {
        return pos_;
    }

    inline Eigen::Map<Eigen::Quaterniond> const &q() const noexcept {
        return rot_;
    }

    inline Eigen::Matrix3d dcm() const noexcept {
        return rot_.toRotationMatrix();
    }


    inline Eigen::Matrix4d matrix() const noexcept {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat.block<3, 3>(0, 0) = rot_.toRotationMatrix();
        mat.block<3, 1>(0, 3) = pos_;
        return mat;
    }

    inline Eigen::Matrix<double, 7, 1> &parameters() {
        return parameters_;
    }

    inline Transform(const Eigen::Vector3d &p, const Eigen::Matrix3d &dcm) noexcept
            : pos_(&parameters_[0]), rot_(&parameters_[3]) {
        pos_ = p;
        rot_ = Eigen::Quaterniond(dcm);
        dcm_ = dcm;
    }

    inline Transform(const Eigen::Vector3d &p, const Eigen::Quaterniond &q) noexcept
            : pos_(&parameters_[0]), rot_(&parameters_[3]) {
        pos_ = p;
        rot_ = q.normalized();
        dcm_ = rot_.toRotationMatrix();
    }

    inline Transform inverse() const {
        return Transform(-(rot_.inverse() * pos_), rot_.inverse());
    }

    inline void Set(const Eigen::Vector3d &p, const Eigen::Matrix3d &dcm) {
        pos_ = p;
        rot_ = Eigen::Quaterniond(dcm).normalized();
        dcm_ = dcm;
    }

    inline void Set(const Eigen::Vector3d &p, const Eigen::Quaterniond &q) {
        pos_ = p;
        rot_ = q.normalized();
        dcm_ = rot_.toRotationMatrix();
    }

    inline void SetIdentity() {
        pos_.setZero();
        rot_.setIdentity();
        dcm_.setIdentity();
    }

    inline Transform operator*(const Transform &other) const {
        return Transform(pos_ + (rot_ * other.pos_), rot_ * other.rot_);
    }

    inline Transform &operator=(const Transform &other) &{
        if (this != &other) {
            this->Set(other.p(), other.q());
        }
        return *this;
    }

    inline Eigen::Vector3d operator*(const Eigen::Vector3d &p) const {
        return rot_ * p + pos_;
    }

    inline double angle(bool is_degree= true) const {
        Eigen::AngleAxisd angle_axis(rot_);
        return is_degree ? angle_axis.angle() * 180.0 / M_PI : angle_axis.angle();
    }

    inline double norm() const {
        return pos_.norm();
    }

    static Transform Random() {
        Eigen::Vector3d p = Eigen::Vector3d::Random();
        Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
        return Transform(p, q);
    }

protected:
    Eigen::Matrix<double, 7, 1> parameters_;
    Eigen::Map<Eigen::Vector3d> pos_;
    Eigen::Map<Eigen::Quaterniond> rot_;
    Eigen::Matrix3d dcm_;

};