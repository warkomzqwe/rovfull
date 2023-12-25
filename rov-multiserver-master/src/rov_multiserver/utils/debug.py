"""
General debug util routines.
"""

from pymessaginglib.Topic import topic_factory

mavlink_dump_topic = topic_factory.create_topic("/vehicle/mavlink/dump/")


def mavlink_dump(data):
    """
    Simple method intended for mavlink data dump via msg bus for analysis.
    """
    mavlink_dump_topic.send(dict(data=data))


