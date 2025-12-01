# BagFileHnadler_ros2.py
import os
from pathlib import Path
from collections import defaultdict

# numpy / opencv
import numpy as np
import cv2

# rosbag2_py + rclpy serialization
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage

# blur_face_manual Cam (keeps your existing Cam API)
from blur_face_manual.Cam import Cam

ros_distro = os.environ.get('ROS_DISTRO')
print(f'ROS_DISTRO: {ros_distro}')


class BagFileHandler_ros2:
    """
    Uses rosbag2_py SequentialReader/SequentialWriter internally while preserving:
    - class name: BagFileHandler_ros2
    - function names / signatures:
        create_reader(path), create_writer(path), get_cams(), export_cams(cams), image_to_compressed_msg(image, header)
    """

    def __init__(self, path, export_folder, camera_topics, passthrough_topics):
        # input bag path (string)
        self.input_bag_path = str(path)

        # Robust handling for empty export_folder: default to current working directory
        if not export_folder:
            export_folder = os.getcwd()
        export_folder = os.path.expanduser(str(export_folder))
        os.makedirs(export_folder, exist_ok=True)

        # Build output bag URI / folder name in same scheme as original: <export_folder>/<input_stem>_blurred
        self.output_bag_name = os.path.join(export_folder, Path(self.input_bag_path).stem + '_blurred')

        # camera topics and passthrough topics (keeps API)
        self.camera_topics = camera_topics or []
        self.passthrough_topics = passthrough_topics or []

        # Detect storage plugin from input bag (sqlite3 vs mcap)
        self.storage_id = self._detect_storage_id(self.input_bag_path)
        print(f'Using storage plugin: {self.storage_id}')

    # ----------------- storage detection -----------------
    def _detect_storage_id(self, uri: str) -> str:
        p = Path(uri)
        if p.is_file():
            if p.suffix.lower() == '.db3':
                return 'sqlite3'
            if p.suffix.lower() == '.mcap':
                return 'mcap'
        if p.is_dir():
            try:
                for child in p.iterdir():
                    if child.suffix.lower() == '.db3':
                        return 'sqlite3'
                    if child.suffix.lower() == '.mcap':
                        return 'mcap'
            except Exception:
                pass
        if '.db3' in uri:
            return 'sqlite3'
        if '.mcap' in uri:
            return 'mcap'
        print('Warning: could not confidently detect storage plugin from path; defaulting to sqlite3')
        return 'sqlite3'

    # ----------------- Reader / Writer creation -----------------
    def create_reader(self, path):
        uri = str(path)
        reader = rosbag2_py.SequentialReader()
        storage_options = StorageOptions(uri=uri, storage_id=self.storage_id)
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        try:
            reader.open(storage_options, converter_options)
        except Exception as e:
            raise RuntimeError(f'Failed to open reader for uri="{uri}" with storage_id="{self.storage_id}": {e}')
        return reader

    def create_writer(self, path):
        uri = str(path)
        writer = rosbag2_py.SequentialWriter()
        storage_options = StorageOptions(uri=uri, storage_id=self.storage_id)
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        try:
            writer.open(storage_options, converter_options)
        except Exception as e:
            raise RuntimeError(f'Failed to open writer for uri="{uri}" with storage_id="{self.storage_id}": {e}')
        return writer

    # ----------------- helper diagnostics -----------------
    def _summarize_bag_topics(self, reader):
        """
        Returns two things:
        - topic_type_map: {topic_name: type_str}
        - counts: {topic_name: message_count}
        This will iterate the bag once (consumes reader), so we reopen a reader copy to count safely in callers.
        """
        counts = defaultdict(int)
        all_topic_types = reader.get_all_topics_and_types()
        topic_type_map = {t.name: t.type for t in all_topic_types}
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            counts[topic] += 1
        return topic_type_map, counts

    # ----------------- Read bag and output cam object -----------------
    def get_cams(self):
        """
        Read camera topics from the input bag and return a list of Cam objects,
        preserving stored compressed image messages and timestamps.
        Signature preserved: get_cams(self)
        """
        # First, open a reader to fetch topic metadata and counts
        reader_for_meta = self.create_reader(self.input_bag_path)
        all_topic_types = reader_for_meta.get_all_topics_and_types()
        topic_type_map = {t.name: t.type for t in all_topic_types}

        # Create a second reader to count message counts
        reader_for_count = self.create_reader(self.input_bag_path)
        _, counts = self._summarize_bag_topics(reader_for_count)
        del reader_for_count

        # Print available topics and their types & counts
        print('Available topics in bag (topic : type) and message counts:')
        for t in all_topic_types:
            cnt = counts.get(t.name, 0)
            print(f'  {t.name} : {t.type}  (messages: {cnt})')

        # Determine effective camera topics:
        bag_topics = set(topic_type_map.keys())
        requested = set(self.camera_topics)
        present_requested = [t for t in self.camera_topics if t in bag_topics]

        effective_camera_topics = []
        if present_requested:
            effective_camera_topics = present_requested
            print(f'Using requested camera topics present in bag: {effective_camera_topics}')
        else:
            # try autodetect: topics whose type is sensor_msgs/msg/CompressedImage
            autodetected = [t for t, ty in topic_type_map.items()
                            if ty == 'sensor_msgs/msg/CompressedImage']
            # also consider topics with "image" or "compressed" in name and >0 messages
            fallback_by_name = [t for t, cnt in counts.items()
                                if (('image' in t.lower() or 'compressed' in t.lower()) and cnt > 0)]
            combined = []
            for t in autodetected + fallback_by_name:
                if t not in combined:
                    combined.append(t)
            if combined:
                effective_camera_topics = combined
                print(f'No requested topics were present. Autodetected camera topics: {effective_camera_topics}')
            else:
                available_topics_list = sorted(list(bag_topics))
                raise RuntimeError(
                    'No camera topics found in bag. '
                    'Requested camera_topics (none present): {}. '.format(self.camera_topics) +
                    'Available topics: {}. See printed counts above.'.format(available_topics_list)
                )

        # Prepare Cam objects in the same order as effective_camera_topics
        cams = [Cam() for _ in range(len(effective_camera_topics))]

        # Iterate the bag and populate cams (open a fresh reader)
        reader = self.create_reader(self.input_bag_path)

        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic not in effective_camera_topics:
                continue

            ith = effective_camera_topics.index(topic)
            msg_type_str = topic_type_map.get(topic, 'sensor_msgs/msg/CompressedImage')
            try:
                msg_type = get_message(msg_type_str)
                msg = deserialize_message(data, msg_type)
            except Exception:
                try:
                    msg = deserialize_message(data, CompressedImage)
                except Exception as e:
                    print(f'Failed to deserialize message on topic {topic} at {timestamp}: {e}')
                    continue

            cams[ith].compressed_imgmsg_list.append(msg)
            cams[ith].timestamp_list.append(timestamp)
            cams[ith].total_frames += 1
            cams[ith].blur_regions.append([])

        # close reader
        del reader

        # Print loaded frame counts; map back to original requested order if necessary
        for i, topic in enumerate(effective_camera_topics):
            print(f'loaded {cams[i].total_frames} frames for {topic}')

        # Update internal camera_topics to effective list so export uses same topics
        self.camera_topics = effective_camera_topics

        return cams

    # ----------------- write both cam and other topics to bag -----------------
    def export_cams(self, cams):
        """
        Write cams and passthrough topics into a new bag (self.output_bag_name).
        Signature preserved: export_cams(self, cams)
        """
        reader = self.create_reader(self.input_bag_path)
        writer = self.create_writer(self.output_bag_name)

        all_topic_types = reader.get_all_topics_and_types()
        topic_type_map = {t.name: t.type for t in all_topic_types}

        # Decide which topics to include: passthrough + camera topics that exist in the bag
        topics_to_write = []
        for tmeta in all_topic_types:
            if tmeta.name in self.passthrough_topics or tmeta.name in self.camera_topics:
                topics_to_write.append(tmeta.name)

        # Create these topics in the writer USING CORRECT TopicMetadata SIGNATURE
        # rosbag2_py.TopicMetadata constructor in some bindings expects (id:int, name:str, type:str, serialization_format:str, ...)
        for topic in topics_to_write:
            typ = topic_type_map.get(topic, 'sensor_msgs/msg/CompressedImage')
            # set id to 0 (rosbag2_py will manage internal ids)
            metadata = TopicMetadata(topic, typ, 'cdr') if ros_distro == "humble" else TopicMetadata(0, topic, typ, 'cdr')
            writer.create_topic(metadata)
            print(f'Added connection {topic} ({typ})')

        def _image_to_compressed_msg(image_np: np.ndarray, header):
            ok, enc = cv2.imencode('.jpg', image_np)
            if not ok:
                raise RuntimeError('cv2.imencode failed')
            new_msg = CompressedImage()
            new_msg.header = header
            new_msg.format = 'jpeg'
            new_msg.data = bytearray(enc.tobytes())
            return new_msg

        # Iterate and write messages (modify camera images when blur_regions exist)
        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic not in topics_to_write:
                continue

            print(f'Writing message of timestamp {timestamp} for topic {topic}')

            if topic in self.camera_topics:
                ith = self.camera_topics.index(topic)
                try:
                    frame_index = cams[ith].timestamp_list.index(timestamp)
                except ValueError:
                    writer.write(topic, data, timestamp)
                    continue

                if cams[ith].blur_regions[frame_index]:
                    orig_type_str = topic_type_map.get(topic, 'sensor_msgs/msg/CompressedImage')
                    try:
                        orig_type = get_message(orig_type_str)
                        orig_msg = deserialize_message(data, orig_type)
                    except Exception:
                        try:
                            orig_msg = deserialize_message(data, CompressedImage)
                        except Exception as e:
                            print(f'Failed to deserialize original image msg on {topic}: {e}')
                            writer.write(topic, data, timestamp)
                            continue

                    new_image = cams[ith].get_image_with_blur(frame_index)
                    new_msg = _image_to_compressed_msg(new_image, orig_msg.header)

                    try:
                        serialized = serialize_message(new_msg)
                        writer.write(topic, serialized, timestamp)
                    except Exception as e:
                        print(f'Failed to serialize/write modified image for topic {topic} at {timestamp}: {e}')
                        writer.write(topic, data, timestamp)
                else:
                    writer.write(topic, data, timestamp)
            else:
                writer.write(topic, data, timestamp)

        del reader
        del writer

        print(f'Bag file written to {self.output_bag_name}')

    def image_to_compressed_msg(self, image, header):
        """
        Preserve the original helper name and signature exactly.
        Returns a sensor_msgs.msg.CompressedImage with numpy->jpeg encoding.
        """
        _, compressed_image = cv2.imencode('.jpg', image)
        return CompressedImage(
            header=header,
            format='jpeg',
            data=np.frombuffer(compressed_image, dtype=np.uint8),
        )
