// acce_1.set_data(acce1);
// acce_2.set_data(acce2);
// a_hp[0] = 1;
// a_hp[1] = -0.9997;
// b_hp[0] = 0.9999;
// b_hp[1] = -0.9999;
//
//
// a_lp[0] = 1;
// a_lp[1] = -0.1584;
// b_lp[0] = 0.5792;
// b_lp[1] = -0.5792;
// acce_1.bandpass_filter(a_lp, b_lp, a_hp, b_hp);
// acce_2.bandpass_filter(a_lp, b_lp, a_hp, b_hp);
//
// imu::Vector<3> acce_1_filtered = acce_1.get_data_BP_filtered();
// imu::Vector<3> acce_2_filtered = acce_2.get_data_BP_filtered();
//
// acce_1.move_variables();
// acce_2.move_variables();




// a_hp[0] = 1;
// a_hp[1] = -0.9752;
// b_hp[0] = 0.9876;
// b_hp[1] = -0.9876;

// pos_1.set_data(acce1, interval_time, acce_1_mag);
// pos_1.set_filter_coeff(a_hp, b_hp);
// pos_1.set_multiplier(1);
// pos_1.calculate_velocity();
// pos_1.calculate_position();
// pos_1.calculate_position_mag();
// pos_1_filtered = pos_1.get_Position_HP_filtered();
// pos_1_mag = pos_1.get_Position_mag();
// pos_1.move_variables();
//
// pos_2.set_data(acce2, interval_time, acce_2_mag);
// pos_2.set_filter_coeff(a_hp, b_hp);
// pos_2.set_multiplier(1);
// pos_2.calculate_velocity();
// pos_2.calculate_position();
// pos_2.calculate_position_mag();
// pos_2_filtered = pos_2.get_Position_HP_filtered();
// pos_2_mag = pos_1.get_Position_mag();
// pos_2.move_variables();



// int countDecimal = 2;
// String stringTwo = "";

//
// // stringTwo = addString_Vector(gyro1, stringTwo, countDecimal, seperator);
// // stringTwo = addString_Vector(acce1, stringTwo, countDecimal, seperator);
// // stringTwo = addString_Vector(gyro2, stringTwo, countDecimal, seperator);
// // stringTwo = addString_Vector(acce2, stringTwo, countDecimal, seperator);
// // stringTwo = addString(status_bno1, stringTwo, seperator);
// // stringTwo = addString(status_bno2, stringTwo, seperator);
//
// stringTwo = addString_Vector(grav1, stringTwo, countDecimal, seperator);
// stringTwo = addString_Vector(grav2, stringTwo, countDecimal, seperator);
//
// String pos_Data = "";


// pos_Data = addString_Array(pos_1_new_HP_filtered, pos_Data, countDecimal, multiplier, seperator);
// pos_Data = addString_Array(pos_2_new_HP_filtered, pos_Data, countDecimal, multiplier, seperator);


//angle_data = addString(String(pos_1_mag*multiplier, 0), angle_data, seperator);
//  angle_data = addString(String(bno2Event.orientation.y, 0), angle_data, seperator);
