atedAhrsMeasurements& preintegratedMeasurements, Vector omegaCoriolis);
  AHRSFactor(size_t rot_i, size_t rot_j, size_t bias,
      const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements, Vector omegaCoriolis,
      const gtsam::Pose3& body_P_sensor);

  // Standard Interface
  gtsam::PreintegratedAhrsMeasurements preintegratedMeasurements() const;
  Vector evaluateError(const gtsam::Rot3& rot_i, const gtsam::Rot3& rot_j,
      Vector bias) const;
  gtsam::Rot3 predict(const gtsam::Rot3& rot_i, Vector bias,
      const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements,
      Vector omegaCoriolis) const;
};

#include <gtsam/navigation/AttitudeFactor.h>
//virtual class AttitudeFactor : gtsam::NonlinearFactor {
//  AttitudeFactor(const Unit3& nZ, const Unit3& bRef);
//  AttitudeFactor();
//};
virtual class Rot3AttitudeFactor : gtsam::NonlinearFactor{
  Rot3AttitudeFactor(size_t key, const gtsam::Unit3& nZ, const gtsam::noiseModel::Diagonal* model,
      const gtsam::Unit3& bRef);
  Rot3AttitudeFactor(size_t key, const gtsam::Unit3& nZ, const gtsam::noiseModel::Diagonal* model);
  Rot3AttitudeFactor();
  void print(string s) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol) const;
  gtsam::Unit3 nZ() const;
  gtsam::Unit3 bRef() const;
};

virtual class Pose3AttitudeFactor : gtsam::NonlinearFactor{
  Pose3AttitudeFactor(size_t key, const gtsam::Unit3& nZ, const gtsam::noiseModel::Diagonal* model,
      const gtsam::Unit3& bRef);
  Pose3AttitudeFactor(size_t key, const gtsam::Unit3& nZ, const gtsam::noiseModel::Diagonal* model);
  Pose3AttitudeFactor();
  void print(string s) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol) const;
  gtsam::Unit3 nZ() const;
  gtsam::Unit3 bRef() const;
};

//*************************************************************************
// Utilities
//*************************************************************************

namespace utilities {

  #include <matlab.h>
  gtsam::KeyList createKeyList(Vector I);
  gtsam::KeyList createKeyList(string s, Vector I);
  gtsam::KeyVector createKeyVector(Vector I);
  gtsam::KeyVector createKeyVector(string s, Vector I);
  gtsam::KeySet createKeySet(Vector I);
  gtsam::KeySet createKeySet(string s, Vector I);
  Matrix extractPoint2(const gtsam::Values& values);
  Matrix extractPoint3(const gtsam::Values& values);
  Matrix extractPose2(const gtsam::Values& values);
  gtsam::Values allPose3s(gtsam::Values& values);
  Matrix extractPose3(const gtsam::Values& values);
  void perturbPoint2(gtsam::Values& values, double sigma, int seed);
  void perturbPose2 (gtsam::Values& values, double sigmaT, double sigmaR, int seed);
  void perturbPoint3(