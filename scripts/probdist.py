#!/usr/bin/env python
import jsonpickle
import sys



for f in sys.argv[1:]:
    plan = None 
    with open(f, "r") as input_file:
        json_data = input_file.read()
        views = jsonpickle.decode(json_data)
        vals = dict()
        count = 0
        y = list()
        for v in views.keys():
            val = views[v]
            y.append(val)
            if val not in vals:
                vals[val] = 0
            vals[val] += 1

        #for val in sorted(vals.keys()):
        #    y.append(vals[val])

    print y
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    import matplotlib.path as path

    fig, ax = plt.subplots()

    # histogram our data with numpy
    data = y
    n, bins = np.histogram(data, len(y))

    left = np.array(bins[:-1])
    right = np.array(bins[1:])
    bottom = np.zeros(len(left))
    top = bottom + n


    # we need a (numrects x numsides x 2) numpy array for the path helper
    # function to build a compound path
    XY = np.array([[left, left, right, right], [bottom, top, top, bottom]]).T

    # get the Path object
    barpath = path.Path.make_compound_path_from_polys(XY)
    
    # make a patch out of it
    patch = patches.PathPatch(
        barpath, facecolor='blue', edgecolor='gray', alpha=0.8)
    ax.add_patch(patch)

    # update the view limits
    ax.set_xlim(left[0], right[-1])
    ax.set_ylim(bottom.min(), top.max())

    plt.show()


            
    import matplotlib.pyplot as plt
    import numpy as np
    import plotly.plotly as py  # tools to communicate with Plotly's server

    numpy_hist = plt.figure()

    plt.hist(x, bins=bins)

    plot_url = py.plot_mpl(numpy_hist, filename='numpy-bins')
        

print "Plan analysis finished."
