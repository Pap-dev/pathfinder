import requests
import pandas as pd
import ast

@staticmethod
def check_internet_connection():
    test_url = "http://www.google.com"
    timeout_span = 5
    is_connected = False

    try:
        requests.get(test_url, timeout = timeout_span)
        is_connected = True
    except (requests.ConnectionError, requests.Timeout) as exception:
        is_connected = False

    return is_connected

class LocationInfos():
    
    def weather_conditions(self, api_key):
        weather_dictionary = {}

        location =  self.vehicle.location
        latitude = location.lat
        longitude = location.lon

        api_url_prefix = "http://api.openweathermap.org/data/2.5/weather?lat="
        api_url = api_url_prefix + str(latitude) + "&lon=" + str(longitude) \
            + "&appid=" + str(api_key)

        response = requests.get(api_url) # https://openweathermap.org/current#geo

        if response.status_code != 200:
            print(response.status_code + ": Could not retrieve info.")
            return
        
        weather_dictionary = ast.literal_eval(response.text)

        return weather_dictionary

    def location_info(self):
        location_dictionary = {}

        location =  self.vehicle.location
        latitude = location.lat
        longitude = location.lon

        api_url = "https://www.metaweather.com/api/location/search/?lattlong=" \
            + str(latitude) + "," + str(longitude)
            
        response = requests.get(api_url) # https://www.metaweather.com/api/

        if response.status_code != 200:
            print(response.status_code + ": Could not retrieve info.")

        location_dictionary = ast.literal_eval(response.text)
        return location_dictionary

    def flight_restrictions(self):
        return
        
    def get_elevation(self):
        location =  self.vehicle.location
        latitude = location.lat
        longitude = location.lon

        query = ('https://api.open-elevation.com/api/v1/lookup'
                f'?locations={latitude},{longitude}')

        response = requests.get(query).json()

        if response.status_code != 200:
            print(response.status_code + ": Could not retrieve info.")

        vehicle_elevation = pd.json_normalize(response, 'results')['elevation'].values[0]

        return vehicle_elevation