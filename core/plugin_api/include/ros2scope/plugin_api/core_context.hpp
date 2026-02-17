#pragma once

namespace ros2scope::event_bus {
class EventBus;
}

namespace ros2scope::layout {
class LayoutManager;
}

namespace ros2scope::ros_bridge {
class RosBridge;
}

namespace ros2scope::type_introspection {
class TypeResolver;
}

namespace ros2scope::workspace_env {
class WorkspaceEnvironment;
}

namespace ros2scope::plugin_api {

struct CoreContext {
  ros2scope::event_bus::EventBus *event_bus{nullptr};
  ros2scope::layout::LayoutManager *layout_manager{nullptr};
  ros2scope::ros_bridge::RosBridge *ros_bridge{nullptr};
  ros2scope::type_introspection::TypeResolver *type_resolver{nullptr};
  ros2scope::workspace_env::WorkspaceEnvironment *workspace_env{nullptr};
};

}  // namespace ros2scope::plugin_api
