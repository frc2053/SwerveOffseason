#include "str/CommandLogitechController.h"

using namespace frc2;

CommandLogitechController::CommandLogitechController(int port)
    : CommandGenericHID(port), m_hid{frc::LogitechController(port)} {}

frc::LogitechController& CommandLogitechController::GetHID() {
  return m_hid;
}

Trigger CommandLogitechController::Thumb(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::kThumb, loop);
}

Trigger CommandLogitechController::TriggerBtn(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::kTrigger, loop);
}

Trigger CommandLogitechController::Three(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k3, loop);
}

Trigger CommandLogitechController::Four(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k4, loop);
}

Trigger CommandLogitechController::Five(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k5, loop);
}

Trigger CommandLogitechController::Six(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k6, loop);
}

Trigger CommandLogitechController::Seven(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k7, loop);
}

Trigger CommandLogitechController::Eight(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k8, loop);
}

Trigger CommandLogitechController::Nine(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k9, loop);
}

Trigger CommandLogitechController::Ten(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k10, loop);
}

Trigger CommandLogitechController::Eleven(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k11, loop);
}

Trigger CommandLogitechController::Twelve(frc::EventLoop* loop) const {
  return Button(frc::LogitechController::Button::k12, loop);
}

Trigger CommandLogitechController::Slider(double threshold,
                                           frc::EventLoop* loop) const {
  return Trigger(loop, [this, threshold] {
    return m_hid.GetSlider() > threshold;
  });
}

double CommandLogitechController::GetX() const {
  return m_hid.GetX();
}

double CommandLogitechController::GetY() const {
  return m_hid.GetY();
}

double CommandLogitechController::GetZ() const {
  return m_hid.GetZ();
}

double CommandLogitechController::GetSlider() const {
  return m_hid.GetSlider();
}