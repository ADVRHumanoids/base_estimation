c_ref = CoM_pos_ref;
v_ref = CoM_desiredVelocityRef;

c_actual = CoM_pos_actual;

% plot pos com
plot(c_ref(1,:)', '--')
hold on;
plot(c_actual(1,:)')

% plot vel com
plot(v_ref(1,:)')
hold on
plot(diff(c_ref(1,:)), '-+')

% plot pos l_sole
plot(l_sole_pos_ref(1,:)', '--')
hold on;
plot(l_sole_pos_actual(1,:)')

% plot vel l_sole
plot(diff(l_sole_pos_actual(3,:)))
hold on
plot(l_sole_desiredTwistRef(3,:), '--')
plot(diff(l_sole_pos_ref(3,:)), '-+')