#pragma once

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include "frc/GenericHID.h"

namespace frc {
class LogitechController : public GenericHID,
                                    public wpi::Sendable,
                                    public wpi::SendableHelper<LogitechController> {
 public:
  explicit LogitechController(int port);

  ~LogitechController() override = default;

  LogitechController(LogitechController&&) = default;
  LogitechController& operator=(LogitechController&&) = default;

  double GetX() const;

  double GetY() const;

  double GetZ() const;

  double GetSlider() const;

  BooleanEvent Slider(double threshold, EventLoop* loop) const;

  BooleanEvent Slider(EventLoop* loop) const;

  bool GetThumbButton() const;
  bool GetThumbButtonPressed();
  bool GetThumbButtonReleased();
  BooleanEvent ThumbButton(EventLoop* loop) const;

  bool GetTriggerButton() const;
  bool GetTriggerButtonPressed();
  bool GetTriggerButtonReleased();
  BooleanEvent TriggerButton(EventLoop* loop) const;

  bool GetThreeButton() const;
  bool GetThreeButtonPressed();
  bool GetThreeButtonReleased();
  BooleanEvent ThreeButton(EventLoop* loop) const;

  bool GetFourButton() const;
  bool GetFourButtonPressed();
  bool GetFourButtonReleased();
  BooleanEvent FourButton(EventLoop* loop) const;

  bool GetFiveButton() const;
  bool GetFiveButtonPressed();
  bool GetFiveButtonReleased();
  BooleanEvent FiveButton(EventLoop* loop) const;

  bool GetSixButton() const;
  bool GetSixButtonPressed();
  bool GetSixButtonReleased();
  BooleanEvent SixButton(EventLoop* loop) const;

  bool GetSevenButton() const;
  bool GetSevenButtonPressed();
  bool GetSevenButtonReleased();
  BooleanEvent SevenButton(EventLoop* loop) const;

  bool GetEightButton() const;
  bool GetEightButtonPressed();
  bool GetEightButtonReleased();
  BooleanEvent EightButton(EventLoop* loop) const;

  bool GetNineButton() const;
  bool GetNineButtonPressed();
  bool GetNineButtonReleased();
  BooleanEvent NineButton(EventLoop* loop) const;

  bool GetTenButton() const;
  bool GetTenButtonPressed();
  bool GetTenButtonReleased();
  BooleanEvent TenButton(EventLoop* loop) const;

  bool GetElevenButton() const;
  bool GetElevenButtonPressed();
  bool GetElevenButtonReleased();
  BooleanEvent ElevenButton(EventLoop* loop) const;

  bool GetTwelveButton() const;
  bool GetTwelveButtonPressed();
  bool GetTwelveButtonReleased();
  BooleanEvent TwelveButton(EventLoop* loop) const;

  struct Button {
    static constexpr int kTrigger = 1;
    static constexpr int kThumb = 2;
    static constexpr int k3 = 3;
    static constexpr int k4 = 4;
    static constexpr int k5 = 5;
    static constexpr int k6 = 6;
    static constexpr int k7 = 7;
    static constexpr int k8 = 8;
    static constexpr int k9 = 9;
    static constexpr int k10 = 10;
    static constexpr int k11 = 11;
    static constexpr int k12 = 12;
  };

  struct Axis {
    static constexpr int x = 0;
    static constexpr int y = 1;
    static constexpr int z = 2;
    static constexpr int slider = 3;
  };

  void InitSendable(wpi::SendableBuilder& builder) override;
};

}  // namespace frc
