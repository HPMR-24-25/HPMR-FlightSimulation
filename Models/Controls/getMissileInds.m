function missileStateInds = getMissileInds()


    missileStateInds.qw = 1;
    missileStateInds.qx = 2;
    missileStateInds.qy = 3;
    missileStateInds.qz = 4;
    missileStateInds.q = [missileStateInds.qw, missileStateInds.qx, missileStateInds.qy, missileStateInds.qz];

    missileStateInds.px_ecef = 5;
    missileStateInds.py_ecef = 6;
    missileStateInds.pz_ecef = 7;
    missileStateInds.pos = [missileStateInds.px_ecef, missileStateInds.py_ecef, missileStateInds.pz_ecef];

    missileStateInds.vx_ecef = 8;
    missileStateInds.vy_ecef = 9;
    missileStateInds.vz_ecef = 10;
    missileStateInds.vel = [missileStateInds.vx_ecef, missileStateInds.vy_ecef, missileStateInds.vz_ecef];
    

    missileStateInds.w_ib_x = 11;
    missileStateInds.w_ib_y = 12;
    missileStateInds.w_ib_z = 13;

    missileStateInds.mass   = 14;

end