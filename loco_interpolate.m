function [t_interp,z_interp] = loco_interpolate(t_all,z_all,fps)
%===================================================================
[m,n] = size(z_all);
t_interp = linspace(t_all(1),t_all(end),fps*(t_all(end)-t_all(1)));

for i=1:n
    z_interp(:,i) = interp1(t_all,z_all(:,i),t_interp);
end
t_interp = t_interp';