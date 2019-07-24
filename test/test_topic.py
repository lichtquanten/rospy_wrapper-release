import rospy
import time
import unittest

from rospywrapper import TopicSource, TopicSink

PKG = 'rospy_wrapper'
NAME = 'test_topic'
TIMEOUT = 20

class TestTopic(unittest.TestCase):

    def test_topic(self):
        from std_msgs.msg import String
	rospy.init_node('test_topic', anonymous=True)
        topic = '/test_topic'
        source = TopicSource(topic, String)
        sink = TopicSink(topic, String)
        with source, sink:
            time.sleep(2)
            test_strs = ['a', 'b', 'c']
            for s in test_strs:
                sink.put(s, rospy.Time.now())
            for msg, t in source:
                self.assertTrue(msg.data in test_strs)
                i = test_strs.index(msg.data)
                test_strs.pop(i)
                if not test_strs:
                    return

if __name__ == '__main__':
        import rosunit
        rosunit.unitrun(PKG, NAME, TestTopic)
