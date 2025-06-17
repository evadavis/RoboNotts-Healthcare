pub(crate) mod crc;
pub(crate) mod message;
pub mod motor_controller;

use motor_controller::*;
use std::{ffi::{c_char, CStr}, os::raw::c_int};

// Public FFI Shims

#[no_mangle]
pub unsafe extern "C" fn motor_controller_new(
    port_path: *const c_char,
    device_address: u8,
) -> *mut MotorController {
    let port_path_cstr = unsafe {
        assert!(!port_path.is_null());
        CStr::from_ptr(port_path)
    };

    if let Ok(port_path_rusty) = port_path_cstr.to_str() {
        if let Ok(mc) = MotorController::new(port_path_rusty, device_address) {
            return  Box::into_raw(Box::new(mc))
        } 
    }

    std::ptr::null_mut::<MotorController>()
}

#[no_mangle]
pub unsafe extern "C" fn motor_controller_free(ptr: *mut MotorController) {
    if ptr.is_null() {
        return;
    }
    drop(unsafe { Box::from_raw(ptr) });
}

#[no_mangle]
pub unsafe extern "C" fn motor_controller_enable_modbus(ptr: *mut MotorController) -> c_int {
    let motor_controller = unsafe {
        if ptr.is_null() {
            return 1;
        }
        &mut *ptr
    };

    if motor_controller.enable_modbus().is_err() {
        1
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn motor_controller_set_motor_enabled(ptr: *mut MotorController) -> c_int {
    let motor_controller = unsafe {
        if ptr.is_null() {
            return 1;
        }
        &mut *ptr
    };

    if motor_controller.set_motor_enabled().is_err() {
        1
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn motor_controller_set_motor_disabled(ptr: *mut MotorController) -> c_int {
    let motor_controller = unsafe {
        if ptr.is_null() {
            return 1;
        }
        &mut *ptr
    };

    if motor_controller.set_motor_disabled().is_err() {
        1
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn motor_controller_get_position(ptr: *mut MotorController, postition_ptr: *mut i32) -> c_int {
    let motor_controller = unsafe {
        if ptr.is_null() {
            return 1;
        }
        &mut *ptr
    };

    if let Ok(value) = motor_controller.get_position() {
        *postition_ptr = value;

        0
    } else {
        1
    }
}

#[no_mangle]
pub unsafe extern "C" fn motor_controller_get_velocity(ptr: *mut MotorController, velocity_ptr: *mut f32) -> c_int {
    let motor_controller = unsafe {
        if ptr.is_null() {
            return 1;
        }
        &mut *ptr
    };

    if let Ok(value) = motor_controller.get_velocity() {
        *velocity_ptr = value;

        0
    } else {
        1
    }
}

pub unsafe extern "C" fn motor_controller_set_velocity(
    ptr: *mut MotorController,
    speed: f32,
) -> c_int {
    let motor_controller = unsafe {
        if ptr.is_null() {
            return 1;
        }
        &mut *ptr
    };

    if motor_controller.set_velocity(speed).is_err() {
        1
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn motor_controller_set_position_feedforward(
    ptr: *mut MotorController,
    ff: i16,
) -> c_int {
    let motor_controller = unsafe {
        if ptr.is_null() {
            return 1;
        }
        &mut *ptr
    };

    if motor_controller.set_position_feedforward(ff).is_err() {
        1
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn motor_controller_set_position_gain(ptr: *mut MotorController, gain: i16) -> c_int {
    let motor_controller = unsafe {
        if ptr.is_null() {
            return 1;
        }
        &mut *ptr
    };

    if motor_controller.set_position_gain(gain).is_err() {
        1
    } else {
        0
    }
}

#[cfg(test)]
mod tests;
