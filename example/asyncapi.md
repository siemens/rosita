# Head AsyncAPI definition 1.0.0 documentation


## Table of Contents

* [Operations](#operations)
  * [SEND /turtle1/color_sensor](#send-turtle1color_sensor-operation)
  * [REPLY /turtlesim/describe_parameters](#reply-turtlesimdescribe_parameters-operation)
  * [REPLY /reset](#reply-reset-operation)
  * [REPLY /turtlesim/get_parameter_types](#reply-turtlesimget_parameter_types-operation)
  * [REPLY /turtlesim/get_parameters](#reply-turtlesimget_parameters-operation)
  * [REPLY /turtlesim/get_type_description](#reply-turtlesimget_type_description-operation)
  * [REPLY /kill](#reply-kill-operation)
  * [REPLY /turtlesim/list_parameters](#reply-turtlesimlist_parameters-operation)
  * [SEND /turtle1/pose](#send-turtle1pose-operation)
  * [REPLY /turtle1/rotate_absolute](#reply-turtle1rotate_absolute-operation)
  * [REPLY /turtlesim/set_parameters](#reply-turtlesimset_parameters-operation)
  * [REPLY /turtlesim/set_parameters_atomically](#reply-turtlesimset_parameters_atomically-operation)
  * [REPLY /turtle1/set_pen](#reply-turtle1set_pen-operation)
  * [REPLY /spawn](#reply-spawn-operation)
  * [REPLY /turtle1/teleport_absolute](#reply-turtle1teleport_absolute-operation)
  * [REPLY /turtle1/teleport_relative](#reply-turtle1teleport_relative-operation)

## Operations

### SEND `/turtle1/color_sensor` Operation

* Operation ID: `Color`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"publisher"` | - | - |

#### Message `Color`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | object | - | - | - | **additional properties are allowed** |
| r | integer | - | - | format (`int16`) | - |
| g | integer | - | - | format (`int16`) | - |
| b | integer | - | - | format (`int16`) | - |

> Examples of payload _(generated)_

```json
{
  "r": 0,
  "g": 0,
  "b": 0
}
```


##### Message tags

| Name | Description | Documentation |
|---|---|---|
| msg | - | - |


### REPLY `/turtlesim/describe_parameters` Operation

* Operation ID: `DescribeParameters`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `DescribeParametersRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| names | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "names": "string"
}
```


#### Response information

* reply will be provided via this designated address: `/turtlesim/describe_parameters`
#### Message `DescribeParametersResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| descriptors | - | - | - | - | **additional properties are allowed** |
| descriptors.name | string | - | - | - | - |
| descriptors.type | integer | - | - | format (`int16`) | - |
| descriptors.description | string | - | - | - | - |
| descriptors.additional_constraints | string | - | - | - | - |
| descriptors.read_only | string | - | - | - | - |
| descriptors.dynamic_typing | string | - | - | - | - |
| descriptors.floating_point_range | - | - | - | - | **additional properties are allowed** |
| descriptors.floating_point_range.from_value | number | - | - | format (`double`) | - |
| descriptors.floating_point_range.to_value | number | - | - | format (`double`) | - |
| descriptors.floating_point_range.step | number | - | - | format (`double`) | - |
| descriptors.integer_range | - | - | - | - | **additional properties are allowed** |
| descriptors.integer_range.from_value | integer | - | - | format (`uint32`) | - |
| descriptors.integer_range.to_value | integer | - | - | format (`uint32`) | - |
| descriptors.integer_range.step | integer | - | - | format (`uint64`) | - |

> Examples of payload _(generated)_

```json
{
  "descriptors": {
    "name": "string",
    "type": 0,
    "description": "string",
    "additional_constraints": "string",
    "read_only": "string",
    "dynamic_typing": "string",
    "floating_point_range": {
      "from_value": 0.1,
      "to_value": 0.1,
      "step": 0.1
    },
    "integer_range": {
      "from_value": 0,
      "to_value": 0,
      "step": 0
    }
  }
}
```




### REPLY `/reset` Operation

* Operation ID: `Empty`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `EmptyRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| structure_needs_at_least_one_member | integer | - | - | format (`int16`) | - |

> Examples of payload _(generated)_

```json
{
  "structure_needs_at_least_one_member": 0
}
```


#### Response information

* reply will be provided via this designated address: `/reset`
#### Message `EmptyResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| structure_needs_at_least_one_member | integer | - | - | format (`int16`) | - |

> Examples of payload _(generated)_

```json
{
  "structure_needs_at_least_one_member": 0
}
```




### REPLY `/turtlesim/get_parameter_types` Operation

* Operation ID: `GetParameterTypes`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `GetParameterTypesRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| names | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "names": "string"
}
```


#### Response information

* reply will be provided via this designated address: `/turtlesim/get_parameter_types`
#### Message `GetParameterTypesResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| types | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "types": "string"
}
```




### REPLY `/turtlesim/get_parameters` Operation

* Operation ID: `GetParameters`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `GetParametersRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| names | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "names": "string"
}
```


#### Response information

* reply will be provided via this designated address: `/turtlesim/get_parameters`
#### Message `GetParametersResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| values | - | - | - | - | **additional properties are allowed** |
| values.type | integer | - | - | format (`int16`) | - |
| values.bool_value | string | - | - | - | - |
| values.integer_value | integer | - | - | format (`uint32`) | - |
| values.double_value | number | - | - | format (`double`) | - |
| values.string_value | string | - | - | - | - |
| values.byte_array_value | string | - | - | - | - |
| values.bool_array_value | string | - | - | - | - |
| values.integer_array_value | string | - | - | - | - |
| values.double_array_value | string | - | - | - | - |
| values.string_array_value | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "values": {
    "type": 0,
    "bool_value": "string",
    "integer_value": 0,
    "double_value": 0.1,
    "string_value": "string",
    "byte_array_value": "string",
    "bool_array_value": "string",
    "integer_array_value": "string",
    "double_array_value": "string",
    "string_array_value": "string"
  }
}
```




### REPLY `/turtlesim/get_type_description` Operation

* Operation ID: `GetTypeDescription`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `GetTypeDescriptionRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| type_name | string | - | - | - | - |
| type_hash | string | - | - | - | - |
| include_type_sources | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "type_name": "string",
  "type_hash": "string",
  "include_type_sources": "string"
}
```


#### Response information

* reply will be provided via this designated address: `/turtlesim/get_type_description`
#### Message `GetTypeDescriptionResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| successful | string | - | - | - | - |
| failure_reason | string | - | - | - | - |
| type_description | - | - | - | - | **additional properties are allowed** |
| type_description.type_description | - | - | - | - | **additional properties are allowed** |
| type_description.type_description.type_name | string | - | - | - | - |
| type_description.type_description.fields | - | - | - | - | **additional properties are allowed** |
| type_description.type_description.fields.name | string | - | - | - | - |
| type_description.type_description.fields.type | - | - | - | - | **additional properties are allowed** |
| type_description.type_description.fields.type.type_id | integer | - | - | format (`int16`) | - |
| type_description.type_description.fields.type.capacity | integer | - | - | format (`uint64`) | - |
| type_description.type_description.fields.type.string_capacity | integer | - | - | format (`uint64`) | - |
| type_description.type_description.fields.type.nested_type_name | string | - | - | - | - |
| type_description.type_description.fields.default_value | string | - | - | - | - |
| type_description.referenced_type_descriptions | - | - | - | - | **additional properties are allowed** |
| type_description.referenced_type_descriptions.type_name | string | - | - | - | - |
| type_description.referenced_type_descriptions.fields | - | - | - | - | **additional properties are allowed** |
| type_description.referenced_type_descriptions.fields.name | string | - | - | - | - |
| type_description.referenced_type_descriptions.fields.type | - | - | - | - | **additional properties are allowed** |
| type_description.referenced_type_descriptions.fields.type.type_id | integer | - | - | format (`int16`) | - |
| type_description.referenced_type_descriptions.fields.type.capacity | integer | - | - | format (`uint64`) | - |
| type_description.referenced_type_descriptions.fields.type.string_capacity | integer | - | - | format (`uint64`) | - |
| type_description.referenced_type_descriptions.fields.type.nested_type_name | string | - | - | - | - |
| type_description.referenced_type_descriptions.fields.default_value | string | - | - | - | - |
| type_sources | - | - | - | - | **additional properties are allowed** |
| type_sources.type_name | string | - | - | - | - |
| type_sources.encoding | string | - | - | - | - |
| type_sources.raw_file_contents | string | - | - | - | - |
| extra_information | - | - | - | - | **additional properties are allowed** |
| extra_information.key | string | - | - | - | - |
| extra_information.value | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "successful": "string",
  "failure_reason": "string",
  "type_description": {
    "type_description": {
      "type_name": "string",
      "fields": {
        "name": "string",
        "type": {
          "type_id": 0,
          "capacity": 0,
          "string_capacity": 0,
          "nested_type_name": "string"
        },
        "default_value": "string"
      }
    },
    "referenced_type_descriptions": {
      "type_name": "string",
      "fields": {
        "name": "string",
        "type": {
          "type_id": 0,
          "capacity": 0,
          "string_capacity": 0,
          "nested_type_name": "string"
        },
        "default_value": "string"
      }
    }
  },
  "type_sources": {
    "type_name": "string",
    "encoding": "string",
    "raw_file_contents": "string"
  },
  "extra_information": {
    "key": "string",
    "value": "string"
  }
}
```




### REPLY `/kill` Operation

* Operation ID: `Kill`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `KillRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| name | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "name": "string"
}
```


#### Response information

* reply will be provided via this designated address: `/kill`
#### Message `KillResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| structure_needs_at_least_one_member | integer | - | - | format (`int16`) | - |

> Examples of payload _(generated)_

```json
{
  "structure_needs_at_least_one_member": 0
}
```




### REPLY `/turtlesim/list_parameters` Operation

* Operation ID: `ListParameters`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `ListParametersRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| prefixes | string | - | - | - | - |
| depth | integer | - | - | format (`uint64`) | - |

> Examples of payload _(generated)_

```json
{
  "prefixes": "string",
  "depth": 0
}
```


#### Response information

* reply will be provided via this designated address: `/turtlesim/list_parameters`
#### Message `ListParametersResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| result | - | - | - | - | **additional properties are allowed** |
| result.names | string | - | - | - | - |
| result.prefixes | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "result": {
    "names": "string",
    "prefixes": "string"
  }
}
```




### SEND `/turtle1/pose` Operation

* Operation ID: `Pose`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"publisher"` | - | - |

#### Message `Pose`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| x | number | - | - | format (`float`) | - |
| y | number | - | - | format (`float`) | - |
| theta | number | - | - | format (`float`) | - |
| linear_velocity | number | - | - | format (`float`) | - |
| angular_velocity | number | - | - | format (`float`) | - |

> Examples of payload _(generated)_

```json
{
  "x": 0.1,
  "y": 0.1,
  "theta": 0.1,
  "linear_velocity": 0.1,
  "angular_velocity": 0.1
}
```



### REPLY `/turtle1/rotate_absolute` Operation

* Operation ID: `RotateAbsolute`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"action_server"` | - | - |

#### Message `RotateAbsoluteRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| theta | number | - | - | format (`float`) | - |

> Examples of payload _(generated)_

```json
{
  "theta": 0.1
}
```


#### Response information

* reply will be provided via this designated address: `/turtle1/rotate_absolute`
Replying with **one of** the following messages:

#### Message `RotateAbsoluteFeedback`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| remaining | number | - | - | format (`float`) | - |

> Examples of payload _(generated)_

```json
{
  "remaining": 0.1
}
```


#### Message `RotateAbsoluteResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| delta | number | - | - | format (`float`) | - |

> Examples of payload _(generated)_

```json
{
  "delta": 0.1
}
```




### REPLY `/turtlesim/set_parameters_atomically` Operation

* Operation ID: `SetParametersAtomically`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `SetParametersAtomicallyRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| parameters | - | - | - | - | **additional properties are allowed** |
| parameters.name | string | - | - | - | - |
| parameters.value | - | - | - | - | **additional properties are allowed** |
| parameters.value.type | integer | - | - | format (`int16`) | - |
| parameters.value.bool_value | string | - | - | - | - |
| parameters.value.integer_value | integer | - | - | format (`uint32`) | - |
| parameters.value.double_value | number | - | - | format (`double`) | - |
| parameters.value.string_value | string | - | - | - | - |
| parameters.value.byte_array_value | string | - | - | - | - |
| parameters.value.bool_array_value | string | - | - | - | - |
| parameters.value.integer_array_value | string | - | - | - | - |
| parameters.value.double_array_value | string | - | - | - | - |
| parameters.value.string_array_value | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "parameters": {
    "name": "string",
    "value": {
      "type": 0,
      "bool_value": "string",
      "integer_value": 0,
      "double_value": 0.1,
      "string_value": "string",
      "byte_array_value": "string",
      "bool_array_value": "string",
      "integer_array_value": "string",
      "double_array_value": "string",
      "string_array_value": "string"
    }
  }
}
```


#### Response information

* reply will be provided via this designated address: `/turtlesim/set_parameters_atomically`
#### Message `SetParametersAtomicallyResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| result | - | - | - | - | **additional properties are allowed** |
| result.successful | string | - | - | - | - |
| result.reason | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "result": {
    "successful": "string",
    "reason": "string"
  }
}
```




### REPLY `/turtlesim/set_parameters` Operation

* Operation ID: `SetParameters`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `SetParametersRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| parameters | - | - | - | - | **additional properties are allowed** |
| parameters.name | string | - | - | - | - |
| parameters.value | - | - | - | - | **additional properties are allowed** |
| parameters.value.type | integer | - | - | format (`int16`) | - |
| parameters.value.bool_value | string | - | - | - | - |
| parameters.value.integer_value | integer | - | - | format (`uint32`) | - |
| parameters.value.double_value | number | - | - | format (`double`) | - |
| parameters.value.string_value | string | - | - | - | - |
| parameters.value.byte_array_value | string | - | - | - | - |
| parameters.value.bool_array_value | string | - | - | - | - |
| parameters.value.integer_array_value | string | - | - | - | - |
| parameters.value.double_array_value | string | - | - | - | - |
| parameters.value.string_array_value | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "parameters": {
    "name": "string",
    "value": {
      "type": 0,
      "bool_value": "string",
      "integer_value": 0,
      "double_value": 0.1,
      "string_value": "string",
      "byte_array_value": "string",
      "bool_array_value": "string",
      "integer_array_value": "string",
      "double_array_value": "string",
      "string_array_value": "string"
    }
  }
}
```


#### Response information

* reply will be provided via this designated address: `/turtlesim/set_parameters`
#### Message `SetParametersResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| results | - | - | - | - | **additional properties are allowed** |
| results.successful | string | - | - | - | - |
| results.reason | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "results": {
    "successful": "string",
    "reason": "string"
  }
}
```




### REPLY `/turtle1/set_pen` Operation

* Operation ID: `SetPen`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `SetPenRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| r | integer | - | - | format (`int16`) | - |
| g | integer | - | - | format (`int16`) | - |
| b | integer | - | - | format (`int16`) | - |
| width | integer | - | - | format (`int16`) | - |
| off | integer | - | - | format (`int16`) | - |

> Examples of payload _(generated)_

```json
{
  "r": 0,
  "g": 0,
  "b": 0,
  "width": 0,
  "off": 0
}
```


#### Response information

* reply will be provided via this designated address: `/turtle1/set_pen`
#### Message `SetPenResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| structure_needs_at_least_one_member | integer | - | - | format (`int16`) | - |

> Examples of payload _(generated)_

```json
{
  "structure_needs_at_least_one_member": 0
}
```




### REPLY `/spawn` Operation

* Operation ID: `Spawn`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `SpawnRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| x | number | - | - | format (`float`) | - |
| y | number | - | - | format (`float`) | - |
| theta | number | - | - | format (`float`) | - |
| name | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "x": 0.1,
  "y": 0.1,
  "theta": 0.1,
  "name": "string"
}
```


#### Response information

* reply will be provided via this designated address: `/spawn`
#### Message `SpawnResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| name | string | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "name": "string"
}
```




### REPLY `/turtle1/teleport_absolute` Operation

* Operation ID: `TeleportAbsolute`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `TeleportAbsoluteRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| x | number | - | - | format (`float`) | - |
| y | number | - | - | format (`float`) | - |
| theta | number | - | - | format (`float`) | - |

> Examples of payload _(generated)_

```json
{
  "x": 0.1,
  "y": 0.1,
  "theta": 0.1
}
```


#### Response information

* reply will be provided via this designated address: `/turtle1/teleport_absolute`
#### Message `TeleportAbsoluteResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| structure_needs_at_least_one_member | integer | - | - | format (`int16`) | - |

> Examples of payload _(generated)_

```json
{
  "structure_needs_at_least_one_member": 0
}
```




### REPLY `/turtle1/teleport_relative` Operation

* Operation ID: `TeleportRelative`

#### `x-ros2` Operation specific information

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| node | - | - | `"/turtlesim"` | - | - |
| role | - | - | `"service_server"` | - | - |

#### Message `TeleportRelativeRequest`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| linear | number | - | - | format (`float`) | - |
| angular | number | - | - | format (`float`) | - |

> Examples of payload _(generated)_

```json
{
  "linear": 0.1,
  "angular": 0.1
}
```


#### Response information

* reply will be provided via this designated address: `/turtle1/teleport_relative`
#### Message `TeleportRelativeResult`

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| (root) | - | - | - | - | **additional properties are allowed** |
| structure_needs_at_least_one_member | integer | - | - | format (`int16`) | - |

> Examples of payload _(generated)_

```json
{
  "structure_needs_at_least_one_member": 0
}
```




