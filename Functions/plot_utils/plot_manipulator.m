function plot_manipulator(man, time)
    L1 = man.L1;
    L2 = man.L2;
    q  = man.q_true_hist(time, :);
    OO = [0; 0; 1];

    RF0 = translation_matrix(man.origin);
    RF1 = RF0 * rotation_matrix(q(1));
    RF2 = RF1 * translation_matrix([L1, 0, 0]) * rotation_matrix(q(2));
    RF3 = RF2 * translation_matrix([L2, 0, 0]) * rotation_matrix(q(3));

    P1 = RF1 * OO;
    P2 = RF2 * OO;
    P3 = RF3 * OO;
    aa = RF3 * [1; 0; 0];
    points = [P1, P2, P3];

    plot(P1(1), P1(2), "kx", "LineWidth", 2, "MarkerSize", 15);
    plot(points(1, :), points(2, :), "-ok", "LineWidth", 2, "MarkerSize", 10);
    quiver(P3(1), P3(2), aa(1), aa(2), "b", "LineWidth", 2, "MaxHeadSize", 0.5);
end
