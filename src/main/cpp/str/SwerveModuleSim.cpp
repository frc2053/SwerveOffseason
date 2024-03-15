#include "str/SwerveModuleSim.h"

using namespace str;

SwerveModuleSim::SwerveModuleSim(
  SwerveModuleConstants constants, 
  SwerveModulePhysical physicalAttrib,
  ctre::phoenix6::sim::TalonFXSimState& driveSimState,
  ctre::phoenix6::sim::TalonFXSimState& steerSimState,
  ctre::phoenix6::sim::CANcoderSimState& steerEncoderSimState
)
  //TODO: fix magic numbers
: driveSim(physicalAttrib.driveMotor, physicalAttrib.driveGearing, 0.025_kg_sq_m),
  steerSim(physicalAttrib.steerMotor, physicalAttrib.steerGearing, 0.004_kg_sq_m),
  driveSimState(driveSimState),
  steerSimState(steerSimState),
  steerEncoderSimState(steerEncoderSimState),
  driveInverted(constants.invertDrive),
  steerInverted(constants.invertSteer),
  driveFrictionVoltage(physicalAttrib.driveFrictionVoltage),
  steerFrictionVoltage(physicalAttrib.steerFrictionVoltage),
  driveGearing(physicalAttrib.driveGearing),
  steerGearing(physicalAttrib.steerGearing)
{

}

void SwerveModuleSim::Update(units::second_t deltaTime, units::volt_t supplyVoltage) {
  driveSimState.Orientation = driveInverted ? ctre::phoenix6::sim::ChassisReference::Clockwise_Positive : ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
  steerSimState.Orientation = steerInverted ? ctre::phoenix6::sim::ChassisReference::Clockwise_Positive : ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
  
  driveSimState.SetSupplyVoltage(supplyVoltage);
  steerSimState.SetSupplyVoltage(supplyVoltage);
  steerEncoderSimState.SetSupplyVoltage(supplyVoltage);

  driveSim.SetInputVoltage(AddFrictionVoltage(driveSimState.GetMotorVoltage(), driveFrictionVoltage));
  steerSim.SetInputVoltage(AddFrictionVoltage(steerSimState.GetMotorVoltage(), steerFrictionVoltage));

  driveSim.Update(deltaTime);
  steerSim.Update(deltaTime);

  driveSimState.SetRawRotorPosition(driveSim.GetAngularPosition() * driveGearing);
  driveSimState.SetRotorVelocity(driveSim.GetAngularVelocity() * driveGearing);

  steerSimState.SetRawRotorPosition(steerSim.GetAngularPosition() * steerGearing);
  steerSimState.SetRotorVelocity(steerSim.GetAngularVelocity() * steerGearing);

  steerEncoderSimState.SetRawPosition(steerSim.GetAngularPosition());
  steerEncoderSimState.SetVelocity(steerSim.GetAngularVelocity());
}

units::volt_t SwerveModuleSim::AddFrictionVoltage(units::volt_t outputVoltage, units::volt_t frictionVoltage) {
  if(units::math::abs(outputVoltage) < frictionVoltage) {
    outputVoltage = 0_V;
  } else if (outputVoltage > 0_V) {
    outputVoltage -= frictionVoltage;
  } else {
    outputVoltage += frictionVoltage;
  }
  return outputVoltage;
}

units::ampere_t SwerveModuleSim::GetSteerCurrentDraw() const {
  return steerSim.GetCurrentDraw();
}

units::ampere_t SwerveModuleSim::GetDriveCurrentDraw() const {
  return driveSim.GetCurrentDraw();
}