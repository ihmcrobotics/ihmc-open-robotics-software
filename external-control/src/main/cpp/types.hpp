#pragma once

#include <Eigen/Core>

namespace ihmc
{
    /** Linear Algebra */
    // DO NOT USE -- FOR INTEROP WITH EJML DMATRIXRMAJ, WE MUST CONSIDER ALL MATRICES TO BE ROW-MAJOR
    // using MatrixView = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::AlignedMax>;
    // using MatrixViewReadOnly = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::AlignedMax>;
    // using RotationView = Eigen::Map<Eigen::Matrix<double, 3, 3>, Eigen::AlignedMax>;
    // using RotationViewReadOnly = Eigen::Map<const Eigen::Matrix<double, 3, 3>, Eigen::AlignedMax>;

    // Alias for mocking a DMatrixRMaj from EJML in API
    using MatrixRowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    // Mutable
    using MatrixViewRowMajor = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::AlignedMax>;
    using VectorView = Eigen::Map<Eigen::Vector<double, Eigen::Dynamic>, Eigen::AlignedMax>;
    // Immutable
    using MatrixViewRowMajorReadOnly = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::AlignedMax>;
    using VectorViewReadOnly = Eigen::Map<const Eigen::Vector<double, Eigen::Dynamic>, Eigen::AlignedMax>;
    // Sized Mutable
    using RotationViewRowMajor = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>, Eigen::AlignedMax>;
    using TranslationView = Eigen::Map<Eigen::Matrix<double, 3, 1>, Eigen::AlignedMax>;
    using SpatialVectorView = Eigen::Map<Eigen::Matrix<double, 6, 1>, Eigen::AlignedMax>;
    // Sized Immutable
    using RotationViewRowMajorReadOnly = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>, Eigen::AlignedMax>;
    using TranslationViewReadOnly = Eigen::Map<const Eigen::Matrix<double, 3, 1>, Eigen::AlignedMax>;
    using SpatialVectorViewReadOnly = Eigen::Map<const Eigen::Matrix<double, 6, 1>, Eigen::AlignedMax>;

    /** Robot */
    // x and y in foot frame
    using ContactSurfaceDimensions = std::pair<double, double>;
    // Kp and Kd for Baumgarte stabilization
    using ContactStabilizationGains = std::pair<double, double>;
}
