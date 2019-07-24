from abc import ABCMeta, abstractmethod
import rosbag
import rospy
import Queue

class Source(object):
    """Base class for source objects.

    Subclasses should implement next().
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    def __iter__(self):
        return self

    @abstractmethod
    def next(self):
        """
        Returns:
            (tuple): tuple containing:
                data: Any data.
                t (rospy.time.Time): A timestamp associated with `data`.
        Raises:
            StopIteration: When there is no more data.
        """
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

class TopicSource(Source):
    """A class that produces data from a ROS topic subscription."""

    def __init__(self, topic, data_class, threadsafe=False):
        """
        It is safe to use a non-threadsafe buffer to store incoming messages
        so long as they are adequately separated in time. For high frequency
        topics, use a threadsafe buffer. Note that the threadsafe buffer will
        introduce a delay for putting and yielding data. See the Queue module
        for details.

        Args:
            topic (str): The resource name of a topic.
            data_class (genpy.Message): Messsage class for serialization of `data.`
                threadsafe (bool): Should the buffer used to store incoming messages
                be threadsafe. Defaults to False.
        """
        self._topic = topic
        self._data_class = data_class
        self._threadsafe = threadsafe
        if threadsafe:
            self._buffer = Queue()
        else:
            self._buffer = []

    def __iter__(self):
        return self

    def next(self):
        """
        Returns only when rospy.is_shutdown() returns True.

        Yields:
            (tuple): tuple containing:
                msg (genpy.Message): The earliest received message on the topic.
                t (rospy.time.Time): The time of receiving msg.
        """
        if self._threadsafe:
            while not rospy.is_shutdown():
                try:
                    return self._buffer.get(block=False)
                except Queue.Empty:
                    rospy.sleep(0.001)
        else:
            while not rospy.is_shutdown():
                if self._buffer:
                    return self._buffer.pop(0)
                else:
                    rospy.sleep(0.001)

    def _callback(self, msg):
        """Put message into buffer.

        Waits for buffer's lock if using `threadsafe` option.

        Args:
            msg (genpy.Message): Message to add to buffer.
        """
        if self._threadsafe:
            self._buffer.put((msg, rospy.get_rostime()))
        else:
            self._buffer.append((msg, rospy.get_rostime()))

    def __enter__(self):
        """Register a subscription to the topic."""
        self._subscriber = rospy.Subscriber(
            self._topic, self._data_class, self._callback)
        return self

    def __exit__(self, *exc):
        """Unregister subscription to the topic."""
        self._subscriber.unregister()

class BagSource(Source):
    """A class that produces data from a bag."""

    def __init__(self, pathname, topic):
        """
        Args:
            topic (str): The name of a topic in the bag.
            pathname (str): A path to a bag file.
        """
        self._pathname = pathname
        self._topic = topic

    def __iter__(self):
        """"""
        return self

    def next(self):
        """
        Returns:
            (tuple): tuple containing:
                msg (genpy.Message): The next message from `topic` in the bag.
                t (rospy.time.Time): The time associated with `msg` in the bag.
        Raises:
            StopIteration: When there are no messages remaining in the bag.
        """
        _, msg, t = next(self._messages)
        return msg, t

    def __enter__(self):
        """Open the bag for reading."""
        self._bag = rosbag.Bag(self._pathname, 'r')
        self._bag.__enter__()
        self._messages = self._bag.read_messages(
            connection_filter=lambda topic, *args: topic == self._topic)
        return self

    def __exit__(self, *exc):
        """Close the bag."""
        self._bag.__exit__(None, None, None)
