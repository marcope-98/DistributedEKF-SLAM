function display_simulation(Robots, profile)
    x_GT = profile.info.Landmarks(:,2:3)';
    for j = 1:profile.sim.nRobots
        x_EST = Robots{j}.LHistory(:,:,end);
        [R, t] = find_transformation(x_EST, x_GT);
        th = atan2(R(2,1), R(1,1));
        Robots{j}.Est(:,2:3) = (R * (Robots{j}.Est(:,2:3)') + t)';
        Robots{j}.Est(:,4) = wrapToPi(Robots{j}.Est(:,4) + th);
    end
    animateMRCLAMdataSet(Robots, profile.info.Landmarks, profile.sim.end, profile.sim.dt);
end