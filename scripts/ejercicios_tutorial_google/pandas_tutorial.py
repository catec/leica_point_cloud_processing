#!/usr/bin/env python


# El siguiente codigo deberia ejecutarse sobre un 

#                   Jupyter Notebook 

# pero no lo he podido configurar adecuadamente para mi Visual Studio Code


from __future__ import print_function

import pandas as pd
pd.__version__

# Primary data types on pandas:
# DataFrame -    which you can imagine as a relational data table, with rows and named columns.
# Series    -    which is a single column. A DataFrame contains one or more Series and a name for each Series.

pd.Series(['San Francisco', 'San Jose', 'Sacramento'])
city_names = pd.Series(['San Francisco', 'San Jose', 'Sacramento'])
population = pd.Series([852469, 1015785, 485199])

# Load some values on DataFrame
pd.DataFrame({ 'City name': city_names, 'Population': population })

# Load an entire file on DataFrame
california_housing_dataframe = pd.read_csv("https://download.mlcc.google.com/mledu-datasets/california_housing_train.csv", sep=",")
california_housing_dataframe.describe() # show some values 
california_housing_dataframe.head()  # show some values 
california_housing_dataframe.hist('housing_median_age') # show graphic

# Access data on pandas list
cities = pd.DataFrame({ 'City name': city_names, 'Population': population })
print(type(cities['City name']))
cities['City name']

# Modify data. Python map function:
population.apply(lambda val: val > 1000000)
# adds two Series to DataFrame
cities['Area square miles'] = pd.Series([46.87, 176.53, 97.92])
cities['Population density'] = cities['Population'] / cities['Area square miles']
# boolean operations with lambda functions
cities['Is wide and has saint name'] = (cities['Area square miles'] > 50) & cities['City name'].apply(lambda name: name.startswith('San'))
cities

# Reorder info
cities.reindex([2, 0, 1])