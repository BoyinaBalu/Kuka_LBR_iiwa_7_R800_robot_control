function limits_verify
load Boyina_BalaSubrahmanyam.txt
q1 = Boyina_BalaSubrahmanyam(:,1);
q2 = Boyina_BalaSubrahmanyam(:,2);
q3 = Boyina_BalaSubrahmanyam(:,3);
q4 = Boyina_BalaSubrahmanyam(:,4);
q5 = Boyina_BalaSubrahmanyam(:,5);
q6 = Boyina_BalaSubrahmanyam(:,6);
q7 = Boyina_BalaSubrahmanyam(:,7);
if (max(abs(q1)>deg2rad(170))||min(abs(q1)<(-170))||max(abs(q2)>deg2rad(120))||min(abs(q2)<(-120))||max(abs(q3)>deg2rad(170))||min(abs(q3)<(-170))||max(abs(q4)>deg2rad(120))||min(abs(q4)<(-120))||max(abs(q5)>deg2rad(170))||min(abs(q5)<(-170))||max(abs(q6)>deg2rad(120))||min(abs(q6)<(-120))||max(abs(q7)>deg2rad(175))||min(abs(q7)<(-175)))
    fprintf("Joint limits reached");
else
    fprintf("Joint limits are in limits");
end

load Velocity_limits_Check.txt
qv1 = Velocity_limits_Check(:,1);
qv2 = Velocity_limits_Check(:,2);
qv3 = Velocity_limits_Check(:,3);
qv4 = Velocity_limits_Check(:,4);
qv5 = Velocity_limits_Check(:,5);
qv6 = Velocity_limits_Check(:,6);
qv7 = Velocity_limits_Check(:,7);
if (max(abs(qv1))>deg2rad(120)||max(abs(qv2))>deg2rad(120)||max(abs(qv3))>deg2rad(100)||max(abs(qv4))>deg2rad(130)||max(abs(qv5))>deg2rad(140)||max(abs(qv6))>deg2rad(180)||max(abs(qv7))>deg2rad(180))
    fprintf("\nvelocity limits reached");
else
    fprintf("\nvelocity limits are in limits");
end
end
