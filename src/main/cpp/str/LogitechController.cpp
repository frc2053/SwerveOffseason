#include "str/LogitechController.h"

#include <wpi/sendable/SendableBuilder.h>

#include "frc/event/BooleanEvent.h"

using namespace frc;

LogitechController::LogitechController(int port) : GenericHID(port) {}

double LogitechController::GetX() const {
    return GetRawAxis(Axis::x);
}

double LogitechController::GetY() const {
    return GetRawAxis(Axis::y);
}

double LogitechController::GetZ() const {
    return GetRawAxis(Axis::z);
}

double LogitechController::GetSlider() const {
    return GetRawAxis(Axis::slider);
}

BooleanEvent LogitechController::Slider(double threshold, EventLoop* loop) const {
    return BooleanEvent(loop, [this, threshold] { return this->GetSlider() > threshold; });
}

BooleanEvent LogitechController::Slider(EventLoop* loop) const {
    return Slider(0.1, loop);
}

bool LogitechController::GetThumbButton() const {
    return GetRawButton(Button::kThumb);
}

bool LogitechController::GetThumbButtonPressed() {
    return GetRawButtonPressed(Button::kThumb);
}

bool LogitechController::GetThumbButtonReleased() {
    return GetRawButtonReleased(Button::kThumb);
}

BooleanEvent LogitechController::ThumbButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetThumbButton(); });
}

bool LogitechController::GetTriggerButton() const {
    return GetRawButton(Button::kTrigger);
}

bool LogitechController::GetTriggerButtonPressed() {
    return GetRawButtonPressed(Button::kTrigger);
}

bool LogitechController::GetTriggerButtonReleased() {
    return GetRawButtonReleased(Button::kTrigger);
}

BooleanEvent LogitechController::TriggerButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetTriggerButton(); });
}

bool LogitechController::GetThreeButton() const {
    return GetRawButton(Button::k3);
}

bool LogitechController::GetThreeButtonPressed() {
    return GetRawButtonPressed(Button::k3);
}

bool LogitechController::GetThreeButtonReleased() {
    return GetRawButtonReleased(Button::k3);
}

BooleanEvent LogitechController::ThreeButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetThreeButton(); });
}

bool LogitechController::GetFourButton() const {
    return GetRawButton(Button::k4);
}

bool LogitechController::GetFourButtonPressed() {
    return GetRawButtonPressed(Button::k4);
}

bool LogitechController::GetFourButtonReleased() {
    return GetRawButtonReleased(Button::k4);
}

BooleanEvent LogitechController::FourButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetFourButton(); });
}

bool LogitechController::GetFiveButton() const {
    return GetRawButton(Button::k5);
}

bool LogitechController::GetFiveButtonPressed() {
    return GetRawButtonPressed(Button::k5);
}

bool LogitechController::GetFiveButtonReleased() {
    return GetRawButtonReleased(Button::k5);
}

BooleanEvent LogitechController::FiveButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetFiveButton(); });
}

bool LogitechController::GetSixButton() const {
    return GetRawButton(Button::k6);
}

bool LogitechController::GetSixButtonPressed() {
    return GetRawButtonPressed(Button::k6);
}

bool LogitechController::GetSixButtonReleased() {
    return GetRawButtonReleased(Button::k6);
}

BooleanEvent LogitechController::SixButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetSixButton(); });
}

bool LogitechController::GetSevenButton() const {
    return GetRawButton(Button::k7);
}

bool LogitechController::GetSevenButtonPressed() {
    return GetRawButtonPressed(Button::k7);
}

bool LogitechController::GetSevenButtonReleased() {
    return GetRawButtonReleased(Button::k7);
}

BooleanEvent LogitechController::SevenButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetSevenButton(); });
}

bool LogitechController::GetEightButton() const {
    return GetRawButton(Button::k8);
}

bool LogitechController::GetEightButtonPressed() {
    return GetRawButtonPressed(Button::k8);
}

bool LogitechController::GetEightButtonReleased() {
    return GetRawButtonReleased(Button::k8);
}

BooleanEvent LogitechController::EightButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetEightButton(); });
}

bool LogitechController::GetNineButton() const {
    return GetRawButton(Button::k9);
}

bool LogitechController::GetNineButtonPressed() {
    return GetRawButtonPressed(Button::k9);
}

bool LogitechController::GetNineButtonReleased() {
    return GetRawButtonReleased(Button::k9);
}

BooleanEvent LogitechController::NineButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetNineButton(); });
}

bool LogitechController::GetTenButton() const {
    return GetRawButton(Button::k10);
}

bool LogitechController::GetTenButtonPressed() {
    return GetRawButtonPressed(Button::k10);
}

bool LogitechController::GetTenButtonReleased() {
    return GetRawButtonReleased(Button::k10);
}

BooleanEvent LogitechController::TenButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetTenButton(); });
}

bool LogitechController::GetElevenButton() const {
    return GetRawButton(Button::k11);
}

bool LogitechController::GetElevenButtonPressed() {
    return GetRawButtonPressed(Button::k11);
}

bool LogitechController::GetElevenButtonReleased() {
    return GetRawButtonReleased(Button::k11);
}

BooleanEvent LogitechController::ElevenButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetElevenButton(); });
}

bool LogitechController::GetTwelveButton() const {
    return GetRawButton(Button::k12);
}

bool LogitechController::GetTwelveButtonPressed() {
    return GetRawButtonPressed(Button::k12);
}

bool LogitechController::GetTwelveButtonReleased() {
    return GetRawButtonReleased(Button::k12);
}

BooleanEvent LogitechController::TwelveButton(EventLoop* loop) const {
    return BooleanEvent(loop, [this]() { return this->GetTwelveButton(); });
}

void LogitechController::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("HID");
  builder.PublishConstString("ControllerType", "Logitech Extreme 3D Pro");
  builder.AddDoubleProperty("X", [this] { return GetX(); }, nullptr);
  builder.AddDoubleProperty("Y", [this] { return GetY(); }, nullptr);
  builder.AddDoubleProperty("Z", [this] { return GetZ(); }, nullptr);
  builder.AddDoubleProperty("Slider", [this] { return GetSlider(); }, nullptr);
  builder.AddBooleanProperty("ThumbButton", [this] { return GetThumbButton(); }, nullptr);
  builder.AddBooleanProperty("TriggerButton", [this] { return GetTriggerButton(); }, nullptr);
  builder.AddBooleanProperty("3", [this] { return GetThreeButton(); }, nullptr);
  builder.AddBooleanProperty("4", [this] { return GetFourButton(); }, nullptr);
  builder.AddBooleanProperty("5", [this] { return GetFiveButton(); }, nullptr);
  builder.AddBooleanProperty("6", [this] { return GetSixButton(); }, nullptr);
  builder.AddBooleanProperty("7", [this] { return GetSevenButton(); }, nullptr);
  builder.AddBooleanProperty("8", [this] { return GetEightButton(); }, nullptr);
  builder.AddBooleanProperty("9", [this] { return GetNineButton(); }, nullptr);
  builder.AddBooleanProperty("10", [this] { return GetTenButton(); }, nullptr);
  builder.AddBooleanProperty("11", [this] { return GetElevenButton(); }, nullptr);
  builder.AddBooleanProperty("12", [this] { return GetTwelveButton(); }, nullptr);
}