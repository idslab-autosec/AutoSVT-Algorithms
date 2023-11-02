# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/common_msgs/dreamview_msgs/hmi_status.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.basic_msgs import header_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2
from modules.common_msgs.monitor_msgs import system_status_pb2 as modules_dot_common__msgs_dot_monitor__msgs_dot_system__status__pb2
from modules.common_msgs.basic_msgs import geometry_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/common_msgs/dreamview_msgs/hmi_status.proto',
  package='apollo.dreamview',
  syntax='proto2',
  serialized_pb=_b('\n3modules/common_msgs/dreamview_msgs/hmi_status.proto\x12\x10\x61pollo.dreamview\x1a+modules/common_msgs/basic_msgs/header.proto\x1a\x34modules/common_msgs/monitor_msgs/system_status.proto\x1a-modules/common_msgs/basic_msgs/geometry.proto\"y\n\x0cScenarioInfo\x12\x13\n\x0bscenario_id\x18\x01 \x01(\t\x12\x15\n\rscenario_name\x18\x02 \x01(\t\x12\x10\n\x08map_name\x18\x03 \x01(\t\x12+\n\x0bstart_point\x18\x04 \x01(\x0b\x32\x16.apollo.common.Point2D\"[\n\x0bScenarioSet\x12\x19\n\x11scenario_set_name\x18\x01 \x01(\t\x12\x31\n\tscenarios\x18\x02 \x03(\x0b\x32\x1e.apollo.dreamview.ScenarioInfo\"\xaa\t\n\tHMIStatus\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\r\n\x05modes\x18\x02 \x03(\t\x12\x14\n\x0c\x63urrent_mode\x18\x03 \x01(\t\x12\x0c\n\x04maps\x18\x04 \x03(\t\x12\x13\n\x0b\x63urrent_map\x18\x05 \x01(\t\x12\x10\n\x08vehicles\x18\x06 \x03(\t\x12\x17\n\x0f\x63urrent_vehicle\x18\x07 \x01(\t\x12\x39\n\x07modules\x18\x08 \x03(\x0b\x32(.apollo.dreamview.HMIStatus.ModulesEntry\x12R\n\x14monitored_components\x18\t \x03(\x0b\x32\x34.apollo.dreamview.HMIStatus.MonitoredComponentsEntry\x12\x14\n\x0c\x64ocker_image\x18\n \x01(\t\x12\x13\n\x0butm_zone_id\x18\x0b \x01(\x05\x12\x15\n\rpassenger_msg\x18\x0c \x01(\t\x12J\n\x10other_components\x18\r \x03(\x0b\x32\x30.apollo.dreamview.HMIStatus.OtherComponentsEntry\x12\x42\n\x0cscenario_set\x18\x0f \x03(\x0b\x32,.apollo.dreamview.HMIStatus.ScenarioSetEntry\x12!\n\x17\x63urrent_scenario_set_id\x18\x10 \x01(\t:\x00\x12\x1d\n\x13\x63urrent_scenario_id\x18\x11 \x01(\t:\x00\x12\x16\n\x0e\x64ynamic_models\x18\x12 \x03(\t\x12\x1d\n\x15\x63urrent_dynamic_model\x18\x13 \x01(\t\x12\x1b\n\x11\x63urrent_record_id\x18\x14 \x01(\t:\x00\x12\x39\n\x07records\x18\x15 \x03(\x0b\x32(.apollo.dreamview.HMIStatus.RecordsEntry\x12\x1c\n\x14\x63urrent_vehicle_type\x18\x16 \x01(\x11\x12%\n\x1d\x63urrent_camera_sensor_channel\x18\x17 \x01(\t\x12#\n\x1b\x63urrent_point_cloud_channel\x18\x18 \x01(\t\x1a.\n\x0cModulesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\x08:\x02\x38\x01\x1a[\n\x18MonitoredComponentsEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12.\n\x05value\x18\x02 \x01(\x0b\x32\x1f.apollo.monitor.ComponentStatus:\x02\x38\x01\x1aW\n\x14OtherComponentsEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12.\n\x05value\x18\x02 \x01(\x0b\x32\x1f.apollo.monitor.ComponentStatus:\x02\x38\x01\x1aQ\n\x10ScenarioSetEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12,\n\x05value\x18\x02 \x01(\x0b\x32\x1d.apollo.dreamview.ScenarioSet:\x02\x38\x01\x1a.\n\x0cRecordsEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\x05:\x02\x38\x01')
  ,
  dependencies=[modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_monitor__msgs_dot_system__status__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_SCENARIOINFO = _descriptor.Descriptor(
  name='ScenarioInfo',
  full_name='apollo.dreamview.ScenarioInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='scenario_id', full_name='apollo.dreamview.ScenarioInfo.scenario_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='scenario_name', full_name='apollo.dreamview.ScenarioInfo.scenario_name', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='map_name', full_name='apollo.dreamview.ScenarioInfo.map_name', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start_point', full_name='apollo.dreamview.ScenarioInfo.start_point', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=219,
  serialized_end=340,
)


_SCENARIOSET = _descriptor.Descriptor(
  name='ScenarioSet',
  full_name='apollo.dreamview.ScenarioSet',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='scenario_set_name', full_name='apollo.dreamview.ScenarioSet.scenario_set_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='scenarios', full_name='apollo.dreamview.ScenarioSet.scenarios', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=342,
  serialized_end=433,
)


_HMISTATUS_MODULESENTRY = _descriptor.Descriptor(
  name='ModulesEntry',
  full_name='apollo.dreamview.HMIStatus.ModulesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.HMIStatus.ModulesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.HMIStatus.ModulesEntry.value', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1271,
  serialized_end=1317,
)

_HMISTATUS_MONITOREDCOMPONENTSENTRY = _descriptor.Descriptor(
  name='MonitoredComponentsEntry',
  full_name='apollo.dreamview.HMIStatus.MonitoredComponentsEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.HMIStatus.MonitoredComponentsEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.HMIStatus.MonitoredComponentsEntry.value', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1319,
  serialized_end=1410,
)

_HMISTATUS_OTHERCOMPONENTSENTRY = _descriptor.Descriptor(
  name='OtherComponentsEntry',
  full_name='apollo.dreamview.HMIStatus.OtherComponentsEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.HMIStatus.OtherComponentsEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.HMIStatus.OtherComponentsEntry.value', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1412,
  serialized_end=1499,
)

_HMISTATUS_SCENARIOSETENTRY = _descriptor.Descriptor(
  name='ScenarioSetEntry',
  full_name='apollo.dreamview.HMIStatus.ScenarioSetEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.HMIStatus.ScenarioSetEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.HMIStatus.ScenarioSetEntry.value', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1501,
  serialized_end=1582,
)

_HMISTATUS_RECORDSENTRY = _descriptor.Descriptor(
  name='RecordsEntry',
  full_name='apollo.dreamview.HMIStatus.RecordsEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.HMIStatus.RecordsEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.HMIStatus.RecordsEntry.value', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1584,
  serialized_end=1630,
)

_HMISTATUS = _descriptor.Descriptor(
  name='HMIStatus',
  full_name='apollo.dreamview.HMIStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.dreamview.HMIStatus.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='modes', full_name='apollo.dreamview.HMIStatus.modes', index=1,
      number=2, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_mode', full_name='apollo.dreamview.HMIStatus.current_mode', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='maps', full_name='apollo.dreamview.HMIStatus.maps', index=3,
      number=4, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_map', full_name='apollo.dreamview.HMIStatus.current_map', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vehicles', full_name='apollo.dreamview.HMIStatus.vehicles', index=5,
      number=6, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_vehicle', full_name='apollo.dreamview.HMIStatus.current_vehicle', index=6,
      number=7, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='modules', full_name='apollo.dreamview.HMIStatus.modules', index=7,
      number=8, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='monitored_components', full_name='apollo.dreamview.HMIStatus.monitored_components', index=8,
      number=9, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='docker_image', full_name='apollo.dreamview.HMIStatus.docker_image', index=9,
      number=10, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='utm_zone_id', full_name='apollo.dreamview.HMIStatus.utm_zone_id', index=10,
      number=11, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='passenger_msg', full_name='apollo.dreamview.HMIStatus.passenger_msg', index=11,
      number=12, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='other_components', full_name='apollo.dreamview.HMIStatus.other_components', index=12,
      number=13, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='scenario_set', full_name='apollo.dreamview.HMIStatus.scenario_set', index=13,
      number=15, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_scenario_set_id', full_name='apollo.dreamview.HMIStatus.current_scenario_set_id', index=14,
      number=16, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_scenario_id', full_name='apollo.dreamview.HMIStatus.current_scenario_id', index=15,
      number=17, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='dynamic_models', full_name='apollo.dreamview.HMIStatus.dynamic_models', index=16,
      number=18, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_dynamic_model', full_name='apollo.dreamview.HMIStatus.current_dynamic_model', index=17,
      number=19, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_record_id', full_name='apollo.dreamview.HMIStatus.current_record_id', index=18,
      number=20, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='records', full_name='apollo.dreamview.HMIStatus.records', index=19,
      number=21, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_vehicle_type', full_name='apollo.dreamview.HMIStatus.current_vehicle_type', index=20,
      number=22, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_camera_sensor_channel', full_name='apollo.dreamview.HMIStatus.current_camera_sensor_channel', index=21,
      number=23, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_point_cloud_channel', full_name='apollo.dreamview.HMIStatus.current_point_cloud_channel', index=22,
      number=24, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_HMISTATUS_MODULESENTRY, _HMISTATUS_MONITOREDCOMPONENTSENTRY, _HMISTATUS_OTHERCOMPONENTSENTRY, _HMISTATUS_SCENARIOSETENTRY, _HMISTATUS_RECORDSENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=436,
  serialized_end=1630,
)

_SCENARIOINFO.fields_by_name['start_point'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT2D
_SCENARIOSET.fields_by_name['scenarios'].message_type = _SCENARIOINFO
_HMISTATUS_MODULESENTRY.containing_type = _HMISTATUS
_HMISTATUS_MONITOREDCOMPONENTSENTRY.fields_by_name['value'].message_type = modules_dot_common__msgs_dot_monitor__msgs_dot_system__status__pb2._COMPONENTSTATUS
_HMISTATUS_MONITOREDCOMPONENTSENTRY.containing_type = _HMISTATUS
_HMISTATUS_OTHERCOMPONENTSENTRY.fields_by_name['value'].message_type = modules_dot_common__msgs_dot_monitor__msgs_dot_system__status__pb2._COMPONENTSTATUS
_HMISTATUS_OTHERCOMPONENTSENTRY.containing_type = _HMISTATUS
_HMISTATUS_SCENARIOSETENTRY.fields_by_name['value'].message_type = _SCENARIOSET
_HMISTATUS_SCENARIOSETENTRY.containing_type = _HMISTATUS
_HMISTATUS_RECORDSENTRY.containing_type = _HMISTATUS
_HMISTATUS.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_HMISTATUS.fields_by_name['modules'].message_type = _HMISTATUS_MODULESENTRY
_HMISTATUS.fields_by_name['monitored_components'].message_type = _HMISTATUS_MONITOREDCOMPONENTSENTRY
_HMISTATUS.fields_by_name['other_components'].message_type = _HMISTATUS_OTHERCOMPONENTSENTRY
_HMISTATUS.fields_by_name['scenario_set'].message_type = _HMISTATUS_SCENARIOSETENTRY
_HMISTATUS.fields_by_name['records'].message_type = _HMISTATUS_RECORDSENTRY
DESCRIPTOR.message_types_by_name['ScenarioInfo'] = _SCENARIOINFO
DESCRIPTOR.message_types_by_name['ScenarioSet'] = _SCENARIOSET
DESCRIPTOR.message_types_by_name['HMIStatus'] = _HMISTATUS

ScenarioInfo = _reflection.GeneratedProtocolMessageType('ScenarioInfo', (_message.Message,), dict(
  DESCRIPTOR = _SCENARIOINFO,
  __module__ = 'modules.common_msgs.dreamview_msgs.hmi_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.ScenarioInfo)
  ))
_sym_db.RegisterMessage(ScenarioInfo)

ScenarioSet = _reflection.GeneratedProtocolMessageType('ScenarioSet', (_message.Message,), dict(
  DESCRIPTOR = _SCENARIOSET,
  __module__ = 'modules.common_msgs.dreamview_msgs.hmi_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.ScenarioSet)
  ))
_sym_db.RegisterMessage(ScenarioSet)

HMIStatus = _reflection.GeneratedProtocolMessageType('HMIStatus', (_message.Message,), dict(

  ModulesEntry = _reflection.GeneratedProtocolMessageType('ModulesEntry', (_message.Message,), dict(
    DESCRIPTOR = _HMISTATUS_MODULESENTRY,
    __module__ = 'modules.common_msgs.dreamview_msgs.hmi_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIStatus.ModulesEntry)
    ))
  ,

  MonitoredComponentsEntry = _reflection.GeneratedProtocolMessageType('MonitoredComponentsEntry', (_message.Message,), dict(
    DESCRIPTOR = _HMISTATUS_MONITOREDCOMPONENTSENTRY,
    __module__ = 'modules.common_msgs.dreamview_msgs.hmi_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIStatus.MonitoredComponentsEntry)
    ))
  ,

  OtherComponentsEntry = _reflection.GeneratedProtocolMessageType('OtherComponentsEntry', (_message.Message,), dict(
    DESCRIPTOR = _HMISTATUS_OTHERCOMPONENTSENTRY,
    __module__ = 'modules.common_msgs.dreamview_msgs.hmi_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIStatus.OtherComponentsEntry)
    ))
  ,

  ScenarioSetEntry = _reflection.GeneratedProtocolMessageType('ScenarioSetEntry', (_message.Message,), dict(
    DESCRIPTOR = _HMISTATUS_SCENARIOSETENTRY,
    __module__ = 'modules.common_msgs.dreamview_msgs.hmi_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIStatus.ScenarioSetEntry)
    ))
  ,

  RecordsEntry = _reflection.GeneratedProtocolMessageType('RecordsEntry', (_message.Message,), dict(
    DESCRIPTOR = _HMISTATUS_RECORDSENTRY,
    __module__ = 'modules.common_msgs.dreamview_msgs.hmi_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIStatus.RecordsEntry)
    ))
  ,
  DESCRIPTOR = _HMISTATUS,
  __module__ = 'modules.common_msgs.dreamview_msgs.hmi_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIStatus)
  ))
_sym_db.RegisterMessage(HMIStatus)
_sym_db.RegisterMessage(HMIStatus.ModulesEntry)
_sym_db.RegisterMessage(HMIStatus.MonitoredComponentsEntry)
_sym_db.RegisterMessage(HMIStatus.OtherComponentsEntry)
_sym_db.RegisterMessage(HMIStatus.ScenarioSetEntry)
_sym_db.RegisterMessage(HMIStatus.RecordsEntry)


_HMISTATUS_MODULESENTRY.has_options = True
_HMISTATUS_MODULESENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_HMISTATUS_MONITOREDCOMPONENTSENTRY.has_options = True
_HMISTATUS_MONITOREDCOMPONENTSENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_HMISTATUS_OTHERCOMPONENTSENTRY.has_options = True
_HMISTATUS_OTHERCOMPONENTSENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_HMISTATUS_SCENARIOSETENTRY.has_options = True
_HMISTATUS_SCENARIOSETENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_HMISTATUS_RECORDSENTRY.has_options = True
_HMISTATUS_RECORDSENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
# @@protoc_insertion_point(module_scope)
