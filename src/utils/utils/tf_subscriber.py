# msg filter
from message_filters import SimpleFilter

class TfSubscriber(SimpleFilter):
    def __init__(self, node, message_type, topic, from_frame, to_frame):
        SimpleFilter.__init__(self)
        self.node = node
        self.message_type = message_type
        self.topic = topic 
        self.from_frame = from_frame
        self.to_frame = to_frame
        self.sub = self.node.create_subscription(self.message_type, self.topic, self.callback, 100)

    def callback(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == self.from_frame and tf.child_frame_id == self.to_frame:
                self.signalMessage(tf)