"""Private action and wheel-command routing for the pallet mission."""

DELIVERY_NAMESPACE = '/pallet/r3/delivery'
PRIVATE_ACTION = f'{DELIVERY_NAMESPACE}/navigate_to_pose'
VELOCITY_TOPICS = ('/cmd_vel_nav', '/cmd_vel_localized', '/cmd_vel_smoothed', '/cmd_vel_safe')
NAVIGATION_OWNERS = {
    '/cmd_vel_nav': ['/behavior_server', '/controller_server'],
    '/cmd_vel_nav_checked': ['/r3_navigation_recovery_gate'],
    '/cmd_vel_smoothed': ['/velocity_smoother'],
    '/cmd_vel_safe': ['/collision_monitor'],
}


def delivery_velocity(topic):
    return f'{DELIVERY_NAMESPACE}/{topic.lstrip("/")}'


def check_navigation_command_owners(node, *, relay=None, navigation):
    def publishers(topic):
        return sorted(f'{p.node_namespace.rstrip("/")}/{p.node_name}'
                      for p in node.get_publishers_info_by_topic(topic))

    for topic in (*VELOCITY_TOPICS, '/cmd_vel_nav_checked'):
        expected = [relay] if topic == '/cmd_vel_safe' and relay is not None else []
        observed = publishers(topic)
        if observed != expected:
            raise ValueError(f'Quyền điều khiển bánh chưa bàn giao trên {topic}: {observed}')
    if navigation:
        for topic, expected in NAVIGATION_OWNERS.items():
            private = delivery_velocity(topic)
            observed = publishers(private)
            # Native behavior plugins can each advertise their own endpoint.
            if set(observed) != set(expected) or (len(expected) == 1 and len(observed) != 1):
                raise ValueError(f'Chuỗi Nav2 chưa sẵn sàng trên {private}: {observed}')
    if publishers('/base_controller/cmd_vel') != ['/cmd_vel_stamper']:
        raise ValueError('Bộ nhận lệnh bánh không thuộc phiên hiện tại.')
