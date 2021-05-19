#ifndef DISCREPANCY_PLANNER_FREE_FLYER_STATE_SPACE_H
#define DISCREPANCY_PLANNER_FREE_FLYER_STATE_SPACE_H

#include <discrepancy_planner/box.h>
#include <ellis_util/search/state_space.h>
using ellis_util::search::StateSpace;

namespace discrepancy_planner {

// State is (x, y, z, yaw, prox. angle, dist. angle)
static constexpr unsigned int kFreeFlyerStateDim = 6;

class FreeFlyerStateSpace : public StateSpace<kFreeFlyerStateDim> {
 public:
  static constexpr unsigned int StateDim = kFreeFlyerStateDim;

  typedef ellis_util::search::State<StateDim> State;
  typedef
    typename ellis_util::search::State<StateDim>::VariableType VariableType;
  typedef std::array<VariableType, StateDim> Variables;
  typedef typename StateSpace<StateDim>::ActionIndex ActionIndex;
  typedef typename StateSpace<StateDim>::CostType CostType;
  typedef typename StateSpace<StateDim>::ProbabilityType ProbabilityType;

  // Converts from continuous to discrete values, and back.
  class Discretizer {
   public:
    Discretizer(const std::array<double, StateDim>& disc =
                  DefaultValueArray<double, StateDim>(0.01));

    int Discretize(double value, unsigned int index) const;

    double Undiscretize(int value, unsigned int index) const;

    const std::array<double, StateDim>& GetDiscretization() const;

    void SetDiscretization(const std::array<double, StateDim>& disc);

   private:
    std::array<double, StateDim> disc_;
  };

  enum Dimensionality { LOW_D = 0, HIGH_D };

  static std::string DimensionalityToStr(Dimensionality dim);

  // TODO HighDimensionalRegion

  struct MotionPrimitive {
    MotionPrimitive(int id = -1,
                    CostType cost = StateSpace<StateDim>::INFINITE_COST,
                    Dimensionality dim = HIGH_D, int vel_x = 0, int vel_y = 0,
                    int vel_z = 0, int vel_yaw = 0, int vel_prox_angle = 0,
                    int vel_dist_angle = 0);

    int id_;
    CostType cost_;
    Dimensionality dim_;

    int vel_x_;
    int vel_y_;
    int vel_z_;
    int vel_yaw_;
    int vel_prox_angle_;
    int vel_dist_angle_;
  };

  enum VariableIndex { X = 0, Y, Z, YAW, PROX_ANGLE, DIST_ANGLE };

  static std::string VariableIndexToStr(VariableIndex index);

  FreeFlyerStateSpace(const Eigen::Matrix<double, 4, 4>& goal_in_world =
                        Eigen::Matrix<double, 4, 4>::Identity(),
                      double goal_dist_thresh = 0.05,
                      double goal_angle_thresh = 0.05,
                      const Discretizer& discretizer = Discretizer());

  virtual ~FreeFlyerStateSpace();

  const Discretizer& GetDiscretizer() const;

  double GetGoalDistThresh() const;

  void SetGoalDistThresh(double thresh);

  double GetGoalAngleThresh() const;

  void SetGoalAngleThresh(double thresh);

  void SetGoalPose(const Eigen::Matrix<double, 4, 4>& goal_in_world);

  void GetGoalPose(double& x, double& y, double& z, double& roll, double& pitch,
                   double& yaw) const;

  bool IsGoalPose(double x, double y, double z, double roll, double pitch,
                  double yaw, double prox_angle, double dist_angle) const;

  bool IsGoal(const Variables& variables) const;

  void GetPose(const Variables& variables, double& x, double& y, double& z,
               double& roll, double& pitch, double& yaw, double& prox_angle,
               double& dist_angle) const;

  State* GetState(double x, double y, double z, double roll, double pitch,
                  double yaw, double prox_angle, double dist_angle);

  std::vector<ActionIndex> GetActions(const State* state) const;

  std::tuple<std::vector<State*>, std::vector<CostType>,
             std::vector<ProbabilityType>>
  GetSucc(const State* state, ActionIndex action);

  bool InCollisionBetween(const Variables& state_variables,
                          const Variables& succ_variables);

  const Box& GetRobotCollisionBody(double x, double y, double z, double roll,
                                   double pitch, double yaw);

  const std::vector<Box>& GetWorldCollisionBodies() const;

  void AddWorldCollisionBody(const Box& body);

  void SetVariableLowerBound(unsigned int index, double bound);

  void SetVariableUpperBound(unsigned int index, double bound);

  bool IsInBounds(unsigned int index, int value);

  const std::vector<MotionPrimitive>& GetMotionPrimitives() const;

  void SetMotionPrimitives(const std::vector<MotionPrimitive>& primitives);

  bool LoadMotionPrimitives(const XmlRpc::XmlRpcValue& param);

 protected:
  Discretizer discretizer_;

  // Goal pose specifications.
  Eigen::Matrix<double, 3, 1> goal_pos_in_world_;
  double goal_roll_;
  double goal_pitch_;
  double goal_yaw_;

  double goal_dist_thresh_;
  double goal_angle_thresh_;

  // For collision checking.
  Box robot_collision_body_;
  std::vector<Box> world_collision_bodies_;

  // Bounds on the state variables.
  std::array<int, StateDim> variable_lower_bound_;
  std::array<int, StateDim> variable_upper_bound_;

  std::vector<MotionPrimitive> motion_primitives_;
};

}  // namespace discrepancy_planner

#endif  // DISCREPANCY_PLANNER_FREE_FLYER_STATE_SPACE_H
