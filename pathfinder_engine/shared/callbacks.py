def wildcard_callback(self, attr_name, value):
    return attr_name, value

def analyze_callbacks(self, vehicle_attributes):
    callbacks_analysis = ""

    return callbacks_analysis

def run_callback_listener(self):
    vehicle_attributes = {
        'version' : '', 
        'location.capabilities' : '', 
        'location.global_frame' : '',
        'location.global_relative_frame' : '', 
        'location.local_frame' : '', 
        'attitude' : '', 
        'velocity' : '', 
        'gps_0' : '', 
        'gimbal' : '',
        'battery' : '', 
        'rangefinder' : '', 
        'ekf_ok' : '', 
        'last_heartbeat' : '', 
        'home_location' : '', 
        'system_status' : '', 
        'heading' : '', 
        'is_armable' : '', 
        'airspeed' : '', 
        'groundspeed' : '', 
        'armed' : '', 
        'mode' : ''
    }

    while 1:
        attribute_name, value = self.vehicle.add_attribute_listener('*', wildcard_callback)

        for attribute in list(vehicle_attributes):
            if attribute != attribute_name:
                continue
            
            vehicle_attributes[attribute_name] = value

        alerts = self.analyze_callbacks(vehicle_attributes)
    
    return alerts, vehicle_attributes

def stop_callback_listener(self):
    self.vehicle.remove_attribute_listener('*', wildcard_callback)