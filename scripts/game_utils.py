# Process filename and builds python dictionary where key is the object name
# and the value is a list of its features. Each line of the text file should
# contain the name of the object along with its characteristics in the
# following format: "object: feature1, feature2, feature3, ..., featureN"
def txtToDict(fn):
    # Create empty dictionary
    objs_dict = {}

    # Open file and read all lines
    with open(fn, 'r') as f:
        lines = f.readlines()

    # For each line
    for l in lines:
        obj, features = l.split(':') # Separate obj name and characteristics
        
        # Split characteristics by comma
        features = features.split(',')
        # Remove whitespaces and newlines
        features = [f.replace(" ", "") for f in features]
        features = [f.replace("\n", "") for f in features]
    
        # Add to dictionary
        objs_dict[obj] = features

    return objs_dict
