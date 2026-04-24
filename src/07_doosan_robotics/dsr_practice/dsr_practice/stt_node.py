import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr


class SttNode(Node):

    def __init__(self):
        super().__init__("stt_node")

        self.declare_parameter("language",          "ko-KR")
        self.declare_parameter("device_index",      -1)
        self.declare_parameter("energy_threshold",  300.0)
        self.declare_parameter("pause_threshold",   0.8)
        self.declare_parameter("phrase_time_limit", 5.0)
        self.declare_parameter("dynamic_energy",    True)
        self.declare_parameter("ambient_duration",  1.0)

        self._lang        = self.get_parameter("language").get_parameter_value().string_value
        self._device_idx  = self.get_parameter("device_index").get_parameter_value().integer_value
        energy_thresh     = self.get_parameter("energy_threshold").get_parameter_value().double_value
        pause_thresh      = self.get_parameter("pause_threshold").get_parameter_value().double_value
        self._phrase_lim  = self.get_parameter("phrase_time_limit").get_parameter_value().double_value
        dynamic_energy    = self.get_parameter("dynamic_energy").get_parameter_value().bool_value
        ambient_duration  = self.get_parameter("ambient_duration").get_parameter_value().double_value

        self._pub = self.create_publisher(String, "/stt_result", 10)

        self._log_devices()

        self._recognizer = sr.Recognizer()
        self._recognizer.energy_threshold = energy_thresh
        self._recognizer.pause_threshold = pause_thresh
        self._recognizer.dynamic_energy_threshold = dynamic_energy

        device = self._device_idx if self._device_idx >= 0 else None
        try:
            self._mic = sr.Microphone(device_index=device)
        except Exception as e:
            self.get_logger().error(f"마이크 열기 실패: {e}")
            raise

        with self._mic as source:
            self.get_logger().info(f"주변 소음 측정 중 ({ambient_duration:.1f}s) ...")
            self._recognizer.adjust_for_ambient_noise(source, duration=ambient_duration)
            self.get_logger().info(
                f"energy_threshold={self._recognizer.energy_threshold:.1f}"
            )

        self._stop_listen = self._recognizer.listen_in_background(
            self._mic, self._on_audio, phrase_time_limit=self._phrase_lim,
        )

        self.get_logger().info(
            f"STT 준비 완료  언어={self._lang}  device_index={self._device_idx}  "
            f"phrase_time_limit={self._phrase_lim:.1f}s"
        )

    def _log_devices(self):
        self.get_logger().info("=== 마이크 장치 목록 ===")
        for idx, name in enumerate(sr.Microphone.list_microphone_names()):
            mark = " ◀" if idx == self._device_idx else ""
            self.get_logger().info(f"  [{idx}] {name}{mark}")

    def _on_audio(self, recognizer: sr.Recognizer, audio: sr.AudioData):
        try:
            text = recognizer.recognize_google(audio, language=self._lang)
        except sr.UnknownValueError:
            return
        except sr.RequestError as e:
            self.get_logger().warning(f"Google STT 요청 실패: {e}")
            return

        text = (text or "").strip()
        if not text:
            return

        self.get_logger().info(f"[STT] {text}")
        msg = String()
        msg.data = text
        self._pub.publish(msg)

    def destroy_node(self):
        stop = getattr(self, "_stop_listen", None)
        if stop is not None:
            try:
                stop(wait_for_stop=False)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
