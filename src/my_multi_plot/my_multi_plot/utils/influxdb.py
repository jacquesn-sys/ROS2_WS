import abc
import collections
from datetime import datetime
import logging
import os
import queue
import threading
import time
import typing
import socket
import traceback
import influxdb_client
from influxdb_client.client import write_api
from influxdb_client.rest import ApiException
from urllib3.exceptions import ReadTimeoutError, NewConnectionError #NameResolutionError,

logger = logging.getLogger(__name__)


class Collector(abc.ABC):
    """
    Abstract Base Class for data collection.
    """

    @abc.abstractmethod
    def record_fields_tags(self, sub_command, fields: dict, tags: dict):
        """Make a point out of given values and record it"""


class InfluxCollector(Collector):
    """
    Entry class to collect sensor data and and send it to Influx DB
    """
    MAX_FAILED_POINTS = 100000

    def __init__(self,
                 tags: dict,
                 influx_config: dict) -> None:
        """
        Create the Collector object

        :param tags: Tags to be be attached to all data on InfluxDB
        :type tags: dict
        :param influx_config: InfluxDB configuration
        :type influx_config: dict
        """
        self.tags = tags

        self.influx_config = influx_config
        self._influx_client = None

        self._point_queue = queue.Queue()
        self._failed_points = []

    def __str__(self):
        return "InfluxCollector"

    @property
    def influx_client(self) -> write_api.WriteApi:
        """
        :return: the influx client we're sending points to
        :rtype: WriteApi
        """
        if self._influx_client is None:
            try:
                client = influxdb_client.InfluxDBClient(
                    url=self.influx_config['url'],
                    token=self.influx_config['token'],
                    enable_gzip=True,
                    verify_ssl=True)
                self._influx_client = client.write_api(write_options=write_api.SYNCHRONOUS)
            except Exception:
                logger.exception('Failed to make an InfluxDBClient.', exc_info=True)
                raise

        return self._influx_client

    def get_all_points(self) -> typing.Iterable[dict]:
        """
        Return all enqueued points
        """
        points = []
        while True:
            try:
                point_dict = self._point_queue.get_nowait()
                if point_dict is None:
                    continue
                points.append(point_dict)
            except queue.Empty:
                return points

    def send_points_in_queue(self) -> None:
        """
        Send points currently in the queue
        """
        points = self.get_all_points()
        points.extend(self._failed_points)
        if not points:
            return
        try:
            self.influx_client.write(
                bucket=self.influx_config['bucket'],
                org=self.influx_config['organization'],
                record=points)
            self._failed_points = []

        except ReadTimeoutError as e:
            print('Timeout occurred while writing to InfluxDB')
            self._failed_points = points[:self.MAX_FAILED_POINTS]
            self._influx_client = None

        except socket.gaierror as e:
            print('DNS resolution failed while writing to InfluxDB')
            self._failed_points = points[:self.MAX_FAILED_POINTS]
            self._influx_client = None

        except NameResolutionError as e:
            print('DNS resolution failed while writing to InfluxDB')
            self._failed_points = points[:self.MAX_FAILED_POINTS]
            self._influx_client = None

        except NewConnectionError as e:
            print('New connection error while writing to InfluxDB')
            self._failed_points = points[:self.MAX_FAILED_POINTS]
            self._influx_client = None

        except ApiException as e:
            if e.status == 422 and "field type conflict" in e.body:
                print('Data type conflict: {}'.format(e.body))
                self._failed_points = [] #points[:self.MAX_FAILED_POINTS]
                self._influx_client = None
            else:
                pass

    def record_measurement(self, point):
        """Record the passed-in datapoint and send it to InfluxDB"""
        if point:
            self._point_queue.put_nowait(point)

    def record_fields_tags(self, measurement: str, fields: dict, tags: dict = None):
        tags = tags or {}
        point_tags = dict(list(self.tags.items()) + list(tags.items()))
        point = {
            'measurement': measurement,
            'time': time.time_ns(),
            'fields': dict(fields),
            'tags': point_tags,
        }
        self.record_measurement(point)

    def send_loop(self):
        while True:
            self.send_points_in_queue()
            time.sleep(1.)

    def start_send_thread(self):
        send_thread = threading.Thread(target=self.send_loop, name="InfluxThread", daemon=True)
        send_thread.start()