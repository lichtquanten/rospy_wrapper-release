import rospy

DATA = 0
TIME = 1

class Synchronizer(object):
    """Merge multiple sources into a single stream, by time.

    The algorithm waits for one message from each source. The latest time
    of these messages is considered the `pivot time`. For each source, the
    algorithm waits for additional messages from the source until a message
    is received with time greater than `pivot time` minus the `inter message
    lower bound` of the source. The algorithm yields a list containing a list
    of messages for each source.

    When a source is exhausted (i.e. raises StopIteration), the algorithm
    stops waiting for messages from this source.

    The algorithm is modeled after but simpler than that of ROS's
    ApproximateTimeFilter.
    """

    def __init__(self, sources, inter_msg_lower_bounds=None):
        """
        Args:
            sources (List[rospywrapper.sink.Sink]): Data sources to be merged.
            inter_msg_lower_bounds (List[float], optional): The minimum time between two
                consecutive messages from a source. Indices correspond with those
                of `sources`. Defaults to None.
        """
        if inter_msg_lower_bounds is None:
            inter_msg_lower_bounds = [0] * len(sources)
        self._source_dicts = [
            {
                'source': sources[i],
                'bound': rospy.Duration(inter_msg_lower_bounds[i]),
                'done': False,
                'buffer': [],
            } for i in xrange(len(sources)) ]
    def __iter__(self):
        return self

    def next(self):
        """
        Yields:
            List[List[(genpy.Message, rospy.time.Time),...],...]: A list
                that contains a list messages, timestamps for each source.
                Sublists default to [] if the source has raised StopIteration.
        Raises:
            StopIteration: When all sources raise StopIteration.
        """
        # Ensure that each source_dict has at least 1 message in its the buffer,
        # except those that are done
        for d in self._source_dicts:
            if d['done']:
                continue
            if not d['buffer']:
                try:
                    d['buffer'].append(next(d['source']))
                except StopIteration:
                    d['done'] = True

        # Terminate if all sources are done
        dones = [d['done'] for d in self._source_dicts]
        if all(dones):
            raise StopIteration

        # Find pivot time
        buffers = [d['buffer'] for d in self._source_dicts if d['buffer']]
        lasts = [buffer[-1] for buffer in buffers]
        last_times = [t for (data, t) in lasts]
        pivot_time = max(last_times)

        output = []

        for d in self._source_dicts:
            if d['done']:
                output.append([])
                continue
            while d['buffer'][-1][TIME] < pivot_time - d['bound']:
                try:
                    d['buffer'].append(next(d['source']))
                except StopIteration:
                    d['done'] = True
                    continue
            if d['done'] or d['buffer'][-1][TIME] <= pivot_time:
                output.append(d['buffer'])
                d['buffer'] = []
            else:
                # Last data in buffer falls after pivot time. Save for
                # next time.
                output.append(d['buffer'][:-1])
                d['buffer'] = [d['buffer'][-1]]
        return output
