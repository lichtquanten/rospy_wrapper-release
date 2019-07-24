# rospy_wrapper

A package that allows easy interchanging of ROS topics, rosbags, and other sources of and sinks for data.

Note: To use ROSTopicSource or ROSTopicSink, there must have been a previous call to rospy.init_node().

## Usage
Instantiate a topic source
```python
from rospywrapper import ROSTopicSource
from sensor_msgs.msg import Image

source = ROSTopicSource(
    topic='/camera/image',
    data_class=Image)
```
Instantiate a rosbag source
```python
from rospywrapper import ROSBagSource

source = ROSBagSource(
    topic='/camera/image',
    filename='input.bag')
```
Use a source to show images
```python
import cv2
import numpy as np

with source:
  for msg, t in source:
    data = np.fromstring(msg['data'], np.uint8)
    img = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
    cv2.imshow(img)
    cv2.waitKey(2)
```
Instantiate a topic sink
```python
from rospywrapper import ROSTopicSink

sink = ROSTopicSink()
```
Instantiate a rosbag sink
```python
from rospywrapper import ROSBagSink

sink = ROSBagSink(filename='output.bag')
```
Write from source to sink
```python
import cv2
import numpy as np

with source, sink:
  for msg, t in source:
    data = np.fromstring(msg['data'], np.uint8)
    img = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    sink.put('/camera/image', Image, gray, t)
```
Create a custom source
```python
import cv2
import rospy

from rospywrapper import Source

class VideoFileSource(Source):
  def __init__(self, filename, start_time=0):
      self.filename = filename
      self.time = start_time
      self.step = None

  def __enter__(self):
      self.video = cv2.VideoCapture(self.filename)
      fps = self.video.get(cv2.CAP_PROP_FPS)
      self.step = 1./fps
      return self

  def __exit__(self, *exc):
      self.video.release()

  def __iter__(self):
      return self

  def next(self):
      if not self.video.isOpened():
          raise StopIteration
      _, frame = self.video.read()
      t = self.time
      self.time += self.step
      return frame, rospy.Time.from_sec(t)
```
Create a custom sink
```python
from rospywrapper import Sink
import csv

class CSVSink(Sink):
    def __init__(self, filename):
        self.filename = filename
        self._file = None
        self._writer = None

    def put(self, topic, data_class, data, t):
        row = [t] + [data[key] for key in data]
        self._writer.writerow(row)

    def __enter__(self):
        self._file = open(self.filename, 'w')
        self._writer = csv.writer(self.file)
        return self

    def __exit__(self, *exc):
        self._file.close()
```
