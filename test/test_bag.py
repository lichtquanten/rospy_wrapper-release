import rosbag
import rospy
import tempfile
import unittest

from rospywrapper import BagSource, BagSink

PKG = 'rospy_wrapper'
NAME = 'test_bag'

class TestTopic(unittest.TestCase):

    def test_bag(self):
        from std_msgs.msg import String
        tmp = tempfile.mktemp()
    	rospy.init_node('test_bag', anonymous=True)
        test_strs = ['a', 'b', 'c']
        times = []
        topic = '/test_bag'
        with rosbag.Bag(tmp, 'w') as bag:
            sink = BagSink(bag, topic, String)
            with sink:
                for s in test_strs:
                    t = rospy.Time.now()
                    times.append(t)
                    sink.put(s, t)
        source = BagSource(tmp, topic)
        with source:
            for msg, t in source:
                self.assertTrue(msg.data in test_strs)
                i = test_strs.index(msg.data)
                self.assertEqual(t.secs, times[i].secs)
                self.assertEqual(t.nsecs, times[i].nsecs)
                test_strs.pop(i)
                times.pop(i)
                if not test_strs:
                    return

if __name__ == '__main__':
        import rosunit
        rosunit.unitrun(PKG, NAME, TestTopic)
