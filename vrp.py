import time
import pandas as pd
from geopy.distance import distance
from geopy.geocoders import Nominatim
import numpy as np


## Simple geocoding with a list of addresses
    # The first address input will be tacitly assumed to be the hub
address_list = ["1273 3rd Ave, New York, NY 10021",
"1-5 E 59th St, New York, NY 10022", 
"405 E 59th St, New York, NY 10022",
"11 W 53rd St, New York, NY 10019",
"601 Lexington Ave, New York, NY 10022", 
"45 Rockefeller Plaza, New York, NY 10111",
"1260 6th Ave, New York, NY 10020",
"601 Lexington Ave, New York, NY 10022"]

def geocode_or_read(geocode = False, address_list = "", filepath = "", wait_time = 2):
    if geocode:
        # Simple geocoder
        def geocode_addresses(address_list, wait_time = wait_time):
            geolocator = Nominatim(user_agent="simple_app")
            lat = []
            long = []
            addr = []
            for address in address_list:

                try:
                    # Just pull the data from the raw geocode object
                    t = geolocator.geocode(address).raw
                    lat.append(t['lat'])
                    long.append(t['lon'])
                    addr.append(t['display_name'])
                    time.sleep(wait_time)
                except:
                    lat.append('NA')
                    long.append('NA')
                    addr.append('NA')
                    time.sleep(wait_time)

            return(pd.DataFrame({'Location':addr, 'Lat':lat, 'Long':long}))

        df = geocode_addresses(address_list, 2)

    # Whether using the geocoder above or using an Excel file, The data must be in the Form:
    # Location --> Whatever data type you want
    # Lat --> Float Latitude
    # Long --> Float Longitude 
    else:
        df = pd.read_excel(filepath)

    return(df)

# Remember to not overquery. Set to default of 2 second wait time
df = geocode_or_read(geocode = True, address_list = address_list, filepath = "", wait_time = 2)



def get_distance_matrix(df):
    # We can make a single list of distances then split them based on length of df into a matrixl ater
    distance_df = []
   
    # Instantiate the base index
    # We'll need to increment the index  as we go
    for loc in df['Location']:
        base_index = df['Location'][df['Location'] == loc].reset_index()['index'][0]
        base = (df.loc[base_index ,]['Lat'], df.loc[base_index ,]['Long'])
           
        for x in range(0,len(df)):
            distance_df.append(  distance(base, (  df.loc[x,]['Lat'], df.loc[x,]['Long']) ).m   )
       
    distance_df = [int(x) for x in distance_df]
    # Get the dimension for the return matrix. 
    # If the original distance df is 64 units, we need to make 8x8 matrix.
        # If original is 81, we need 9x9 matrix, etc.
        # So we can simply take the sqrt of the length to get each dimension 
    dim_size = int(len(distance_df)**0.5) 
    distance_matrix = np.array(distance_df).reshape( dim_size, dim_size )

    # For input to Google's ORTools, we need a list of listsdata structure
    list_of_lists = []
    for i in range(len(distance_matrix)):
        list_of_lists.append(list(distance_matrix[i]))

    return ( list_of_lists )


distance_matrix = get_distance_matrix(df)


"""Simple Vehicles Routing Problem (VRP).

   This is a sample using the routing library python wrapper to solve a VRP
   problem.
   A description of the problem can be found here:
   http://en.wikipedia.org/wiki/Vehicle_routing_problem.

   Distances are in meters.
"""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model(distance_matrix, num_vehicles, depot):
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = num_vehicles
    data['depot'] = depot
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))



def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model(distance_matrix = distance_matrix, num_vehicles = 4, depot = 0)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # Slack variables are for waiting time. If it doesn't matter how long vehicles wait, just set this to a huge number. If no slack, 0.
        100000,  # vehicle maximum travel distance. There must be a decent way to parameterize this so it's not always manual but I think that's a business decision.
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print('No solution found !')


if __name__ == '__main__':
    main()