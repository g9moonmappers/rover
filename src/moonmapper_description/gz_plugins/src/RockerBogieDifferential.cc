// Rocker-bogie differential plugin for Gazebo Sim

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <string>

namespace moonmapper {

static std::string ToLower(std::string s)
{ 
  for (char &c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

class RockerBogieDifferential : public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate {
public:
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->model_ = gz::sim::Model(_entity);
    if (!this->model_.Valid(_ecm)) {
      gzerr << "[RockerBogieDifferential] Plugin must be attached to a <model>\n";
      return;
    }

    this->left_joint_name_ =
        _sdf->Get<std::string>("left_rocker_joint", std::string{"rocker_left_joint"}).first;
    this->right_joint_name_ =
        _sdf->Get<std::string>("right_rocker_joint", std::string{"rocker_right_joint"}).first;
    this->diff_joint_name_ =
        _sdf->Get<std::string>("diff_joint", std::string{"rocker_bogie_diff_joint"}).first;
    this->bogie_l_name_ =
        _sdf->Get<std::string>("left_bogie_joint", std::string{"bogie_left_joint"}).first;
    this->bogie_r_name_ =
        _sdf->Get<std::string>("right_bogie_joint", std::string{"bogie_right_joint"}).first;

    this->enable_ = _sdf->Get<bool>("enable", true).first;
    this->debug_ = _sdf->Get<bool>("debug", false).first;
    this->print_interval_ = std::max(0.05, _sdf->Get<double>("print_interval", 0.5).first);

    const std::string mode_raw =
        _sdf->Get<std::string>("coupling_mode", std::string{"weak"}).first;
    this->coupling_mode_ = ToLower(mode_raw);
    this->right_sign_ = _sdf->Get<double>("right_sign", 1.0).first;
    if (std::abs(this->right_sign_) < 0.5) {
      this->right_sign_ = 1.0;
    }
    this->target_sum_ = _sdf->Get<double>("target_sum", 0.0).first;
    this->k_diff_L_ = _sdf->Get<double>("k_diff_L", -0.41).first;
    this->k_diff_R_ = _sdf->Get<double>("k_diff_R", 0.41).first;
    this->k_diff_0_ = _sdf->Get<double>("k_diff_0", 0.0).first;

    if (this->coupling_mode_ == "off") {
      this->walk_beam_ = false;
      this->diff_bar_ = false;
      this->kp_ = 0.0;
      this->kd_ = 0.0;
      this->max_torque_ = 0.0;
      this->kp_aux_ = 0.0;
      this->kd_aux_ = 0.0;
      this->max_torque_aux_ = 0.0;
    } else if (this->coupling_mode_ == "weak") {
      this->walk_beam_ = true;
      this->diff_bar_ = true;
      this->kp_ = 3.5;
      this->kd_ = 0.55;
      this->max_torque_ = 0.85;
      this->kp_aux_ = 1.8;
      this->kd_aux_ = 0.35;
      this->max_torque_aux_ = 0.12;
    } else if (this->coupling_mode_ == "full") {
      this->walk_beam_ = true;
      this->diff_bar_ = true;
      this->kp_ = _sdf->Get<double>("kp", 22.0).first;
      this->kd_ = _sdf->Get<double>("kd", 5.5).first;
      this->max_torque_ = _sdf->Get<double>("max_torque", 1.8).first;
      this->kp_aux_ = _sdf->Get<double>("kp_aux", 5.0).first;
      this->kd_aux_ = _sdf->Get<double>("kd_aux", 1.2).first;
      this->max_torque_aux_ = _sdf->Get<double>("max_torque_aux", 0.35).first;
    } else {
      gzwarn << "[RockerBogieDifferential] Ukjent coupling_mode='" << mode_raw
             << "', bruker 'weak'. Tillatt: off | weak | full\n";
      this->coupling_mode_ = "weak";
      this->walk_beam_ = true;
      this->diff_bar_ = true;
      this->kp_ = 3.5;
      this->kd_ = 0.55;
      this->max_torque_ = 0.85;
      this->kp_aux_ = 1.8;
      this->kd_aux_ = 0.35;
      this->max_torque_aux_ = 0.12;
    }

    this->last_print_s_ = -1.0;
    gzmsg << "[RockerBogieDifferential] model='" << this->model_.Name(_ecm) << "' mode='"
           << this->coupling_mode_ << "' enable=" << (this->enable_ ? 1 : 0) << " walk_beam="
           << (this->walk_beam_ ? 1 : 0) << " diff_bar=" << (this->diff_bar_ ? 1 : 0)
           << " debug=" << (this->debug_ ? 1 : 0) << " | sum-PD kp=" << this->kp_
           << " kd=" << this->kd_ << " max_tau=" << this->max_torque_
           << " sign=" << this->right_sign_ << " | q_diff*=" << this->k_diff_L_ << "*qL+"
           << this->k_diff_R_ << "*qR+" << this->k_diff_0_ << "\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->enable_ || _info.paused) {
      return;
    }

    const double sim_s = std::chrono::duration<double>(_info.simTime).count();

    CacheJoint(_ecm, this->left_joint_name_, this->left_joint_);
    CacheJoint(_ecm, this->right_joint_name_, this->right_joint_);
    if (!this->diff_joint_name_.empty()) {
      CacheJoint(_ecm, this->diff_joint_name_, this->diff_joint_);
    } else {
      this->diff_joint_ = gz::sim::kNullEntity;
    }
    CacheJoint(_ecm, this->bogie_l_name_, this->bogie_l_joint_);
    CacheJoint(_ecm, this->bogie_r_name_, this->bogie_r_joint_);

    if (this->left_joint_ == gz::sim::kNullEntity || this->right_joint_ == gz::sim::kNullEntity) {
      if (!this->warned_missing_) {
        gzwarn << "[RockerBogieDifferential] Mangler rocker-ledd.\n";
        this->warned_missing_ = true;
      }
      return;
    }

    EnsureJointComponents(_ecm, this->left_joint_);
    EnsureJointComponents(_ecm, this->right_joint_);
    if (this->diff_joint_ != gz::sim::kNullEntity) {
      EnsureJointComponents(_ecm, this->diff_joint_);
    }
    if (this->bogie_l_joint_ != gz::sim::kNullEntity) {
      EnsureJointComponents(_ecm, this->bogie_l_joint_);
    }
    if (this->bogie_r_joint_ != gz::sim::kNullEntity) {
      EnsureJointComponents(_ecm, this->bogie_r_joint_);
    }

    double qL = 0.0, qR = 0.0, dqL = 0.0, dqR = 0.0;
    if (!ReadJointState(_ecm, this->left_joint_, qL, dqL) ||
        !ReadJointState(_ecm, this->right_joint_, qR, dqR)) {
      return;
    }

    double q_bl = 0.0, dq_bl = 0.0, q_br = 0.0, dq_br = 0.0;
    const bool have_bl = this->bogie_l_joint_ != gz::sim::kNullEntity &&
                         ReadJointState(_ecm, this->bogie_l_joint_, q_bl, dq_bl);
    const bool have_br = this->bogie_r_joint_ != gz::sim::kNullEntity &&
                         ReadJointState(_ecm, this->bogie_r_joint_, q_br, dq_br);
    (void)have_bl;
    (void)have_br;

    double qd = 0.0, dqd = 0.0;
    const bool have_diff = this->diff_joint_ != gz::sim::kNullEntity &&
                           ReadJointState(_ecm, this->diff_joint_, qd, dqd);

    double tau_L = 0.0;
    double tau_R = 0.0;
    double tau_d = 0.0;

    // Walking beam: hold qL + sign*qR nærmest target_sum (erstatter lukket URDF-lokke i sim).
    if (this->walk_beam_) {
      const double s = this->right_sign_;
      const double err = (qL + s * qR) - this->target_sum_;
      const double err_dot = (dqL + s * dqR);
      const double tau_cmd = -this->kp_ * err - this->kd_ * err_dot;
      tau_L = std::clamp(tau_cmd, -this->max_torque_, this->max_torque_);
      tau_R = s * tau_L;
    }

    // Diff-stav: dytter diff_joint mot lineær kombinasjon av rocker vinkler.
    if (this->diff_bar_ && this->diff_joint_ != gz::sim::kNullEntity && have_diff) {
      const double q_star = this->k_diff_L_ * qL + this->k_diff_R_ * qR + this->k_diff_0_;
      tau_d = -this->kp_aux_ * (qd - q_star) - this->kd_aux_ * dqd;
      tau_d = std::clamp(tau_d, -this->max_torque_aux_, this->max_torque_aux_);
    }

    _ecm.SetComponentData<gz::sim::components::JointForceCmd>(this->left_joint_, {tau_L});
    _ecm.SetComponentData<gz::sim::components::JointForceCmd>(this->right_joint_, {tau_R});
    if (this->diff_joint_ != gz::sim::kNullEntity) {
      _ecm.SetComponentData<gz::sim::components::JointForceCmd>(this->diff_joint_, {tau_d});
    }

    if (this->debug_ &&
        (this->last_print_s_ < 0.0 || sim_s - this->last_print_s_ >= this->print_interval_)) {
      this->last_print_s_ = sim_s;
      const double sum = qL + this->right_sign_ * qR;
      gzmsg << "[RockerBogieDiff] t=" << sim_s << " mode=" << this->coupling_mode_
            << " qL=" << qL << " qR=" << qR << " dqL=" << dqL << " dqR=" << dqR << " sumLR=" << sum
            << " q_bl=" << q_bl << " q_br=" << q_br << " q_diff=" << qd << " tau_L=" << tau_L
            << " tau_R=" << tau_R << " tau_diff=" << tau_d << "\n";
    }
  }

private:
  template<typename T>
  static void EnsureComponent(
      gz::sim::EntityComponentManager &_ecm, const gz::sim::Entity &_entity)
  {
    if (!_ecm.Component<T>(_entity)) {
      _ecm.CreateComponent(_entity, T());
    }
  }

  static void EnsureJointComponents(
      gz::sim::EntityComponentManager &_ecm, const gz::sim::Entity &_entity)
  {
    EnsureComponent<gz::sim::components::JointPosition>(_ecm, _entity);
    EnsureComponent<gz::sim::components::JointVelocity>(_ecm, _entity);
  }

  void CacheJoint(
      gz::sim::EntityComponentManager &_ecm, const std::string &_name, gz::sim::Entity &_entity)
  {
    if (_entity == gz::sim::kNullEntity && !_name.empty()) {
      _entity = this->model_.JointByName(_ecm, _name);
    }
  }

  static bool ReadJointState(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_entity,
      double &_q,
      double &_dq)
  {
    const auto *pos = _ecm.Component<gz::sim::components::JointPosition>(_entity);
    const auto *vel = _ecm.Component<gz::sim::components::JointVelocity>(_entity);
    if (!pos || !vel || pos->Data().empty() || vel->Data().empty()) {
      return false;
    }
    _q = pos->Data()[0];
    _dq = vel->Data()[0];
    return true;
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string left_joint_name_;
  std::string right_joint_name_;
  std::string diff_joint_name_;
  std::string bogie_l_name_;
  std::string bogie_r_name_;
  gz::sim::Entity left_joint_{gz::sim::kNullEntity};
  gz::sim::Entity right_joint_{gz::sim::kNullEntity};
  gz::sim::Entity diff_joint_{gz::sim::kNullEntity};
  gz::sim::Entity bogie_l_joint_{gz::sim::kNullEntity};
  gz::sim::Entity bogie_r_joint_{gz::sim::kNullEntity};
  std::string coupling_mode_{"weak"};
  bool walk_beam_{true};
  bool diff_bar_{true};
  double kp_{3.5};
  double kd_{0.55};
  double target_sum_{0.0};
  double max_torque_{0.85};
  double right_sign_{1.0};
  double k_diff_L_{-0.41};
  double k_diff_R_{0.41};
  double k_diff_0_{0.0};
  double kp_aux_{1.8};
  double kd_aux_{0.35};
  double max_torque_aux_{0.12};
  bool enable_{true};
  bool debug_{false};
  double print_interval_{0.5};
  double last_print_s_{-1.0};
  bool warned_missing_{false};
};

}  // namespace moonmapper

GZ_ADD_PLUGIN(
    moonmapper::RockerBogieDifferential,
    gz::sim::System,
    moonmapper::RockerBogieDifferential::ISystemConfigure,
    moonmapper::RockerBogieDifferential::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(moonmapper::RockerBogieDifferential, "moonmapper::RockerBogieDifferential")
