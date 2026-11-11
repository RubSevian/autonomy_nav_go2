"""Wire parity for the atomic NAV command envelope; no ROS graph or actuators."""
import unittest
from rclpy.serialization import serialize_message,deserialize_message
from go2_navigation_interfaces.msg import NavigationReadiness

class Tests(unittest.TestCase):
    def test_envelope_preserves_original_time_and_session(self):
        for enabled in (False,True):
            source=dict(observed_ns=9_000_000_001,session_id='nav-session',navigation_ready=True,
                        perception_ready=False,command_enabled=enabled,command=[.1,-.2,.03],command_age_s=.15)
            msg=deserialize_message(serialize_message(NavigationReadiness(**source)),NavigationReadiness)
            self.assertEqual(msg.schema_version,1)
            for key,value in source.items():
                self.assertEqual(list(getattr(msg,key)) if key=='command' else getattr(msg,key),value)
    def test_maximum_session_survives_wire(self):
        msg=NavigationReadiness(session_id='x'*128)
        result=deserialize_message(serialize_message(msg),NavigationReadiness)
        self.assertEqual(result.session_id,msg.session_id)
        # Python assignment need not enforce IDL bounds. R3's admission gate
        # independently rejects empty/oversized sessions, as for JSON.
if __name__=='__main__':unittest.main()
