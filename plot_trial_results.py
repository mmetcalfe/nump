import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from operator import itemgetter
import io
import yaml
import os.path
import argparse
import math

# loadYamlFile :: String -> IO (Tree String)
def loadResultsTable(fname):
    if not os.path.isfile(fname):
        raise ValueError('Input file \'{}\' does not exist!'.format(fname))
    file = open(fname, 'r')
    data = yaml.load(file)
    file.close()
    return data

# results_table: [
#     trial_result: {
#         trial_name: 'sdfdsf'
#         skipFrac: 0.66,
#         posFrac: 0.5,
#         ...
#         positive_test_set: {
#             objects: 32,
#             detections: 32,
#             hit_count: 32,
#         }
#         neg_test_set: {
#             ...
#         }
#     }
# ]

def isHAAR(trial):
    return trial['featureType'] == 'HAAR'
def isHOG(trial):
    return trial['featureType'] == 'HOG'
def isLBP(trial):
    return trial['featureType'] == 'LBP'

def partitionOn(key, results):
    parts = {}
    for r in results:
        v = r[key]
        if v in parts:
            parts[v] += [r]
        else:
            parts[v] = [r]
    return parts

def plotFailureSuccessRates(results_table):

    font = {
    # 'family' : 'normal',
            # 'weight' : 'bold',
            'size'   : 22}

    matplotlib.rc('font', **font)
    matplotlib.rc('text', usetex=True)

    from matplotlib.font_manager import FontProperties
    smallFontP = FontProperties()
    smallFontP.set_size('small')

    ind_labels = sorted(results_table.keys()) #map(lambda s: '{:2d}'.format(int(round(100*float(s)))), sorted(ind_parts.keys()))

    sortedResults = []
    for key in ind_labels:
        sortedResults += [results_table[key]]

    N = len(results_table.keys())
    print N
    ind = np.arange(N)  # the x locations for the groups
    width = 0.25         # the width of the bars

    series_cols = ['#fc8d59', '#ffffbf', '#91bfdb']

    for value_key in ['failure_success']:
        fig, ax = plt.subplots()

        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')

        def autolabel(rects):
            # attach some text labels
            for rect in rects:
                height = rect.get_height()
                ax.text(rect.get_x()+rect.get_width()/2., height + width*0.025, '{:.2f}'.format(float(height)),
                        ha='center', va='bottom',
                        fontsize=7)
                        # fontsize=13)

        resultsKeys = ['kickSuccessRate', 'kickFailureRate', 'collisionFailureRate']

        # series_x_offset = 0
        series_x_offset = -width*0.5
        series_num = 0
        part_rects_dict = {}
        for resultsKey in resultsKeys:
            values = map(itemgetter(resultsKey), sortedResults)
            print values
            series_rects = ax.bar(ind + series_x_offset, values, width, color=series_cols[series_num])
            series_x_offset += width
            series_num += 1
            autolabel(series_rects)
            part_rects_dict[resultsKey] = series_rects

        ax.set_ylabel('Success / Failure Rate (\%)')
        ax.set_xlabel('Chance Constraint. \%')
        ax.set_xticks(ind+width)
        ax.set_xticklabels(ind_labels)

        legendHeadings = ['Kick Success', 'Kick Failure', 'Collision Failure']
        sorted_part_keys = resultsKeys # sorted(part_rects_dict.keys())

        ax.legend([part_rects_dict[key] for key in sorted_part_keys]
        , legendHeadings
        , ncol=3
        , prop=smallFontP
        , loc='upper center'
        , bbox_to_anchor=(0.5, 1.15))

        plt.savefig('results_fig_{}.pdf'.format(value_key), bbox_inches='tight')

def plotResultsTable(results_table):

    font = {
    # 'family' : 'normal',
            # 'weight' : 'bold',
            'size'   : 22}

    matplotlib.rc('font', **font)
    matplotlib.rc('text', usetex=True)

    from matplotlib.font_manager import FontProperties
    smallFontP = FontProperties()
    smallFontP.set_size('small')

    # Varied the following:
    #   - number      (3)
    #   - hardNegFrac (2)
    #   - featureType (3)
    #   - skipFrac    (2)

    # haar_results = filter(isHAAR, results_table.values())
    # hog_results = filter(isHOG, results_table.values())
    # lbp_results = filter(isLBP, results_table.values())
    # parts = partitionOn('skipFrac', lbp_results)

    # womenMeans = (25, 32, 34, 20, 25)
    # womenStd =   (3, 5, 2, 3, 3)
    # rects1 = ax.bar(ind, menMeans, width, color='r', yerr=menStd)

    # series_parts = {}
    # series_parts['rrbt'] = results_table

    # ind_parts = partitionOn('chanceConstraint', results_table)
    ind_labels = sorted(results_table.keys()) #map(lambda s: '{:2d}'.format(int(round(100*float(s)))), sorted(ind_parts.keys()))

    sortedResults = []
    for key in ind_labels:
        sortedResults += [results_table[key]]

    N = len(results_table.keys())
    print N
    ind = np.arange(N)  # the x locations for the groups
    width = 0.25         # the width of the bars

    series_cols = ['#fc8d59', '#ffffbf', '#91bfdb']

    for value_key in ['times']:
        fig, ax = plt.subplots()

        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')

        def autolabel(rects):
            # attach some text labels
            for rect in rects:
                height = rect.get_height()
                ax.text(rect.get_x()+rect.get_width()/2., height + width*0.025, '{:.2f}'.format(float(height)),
                        ha='center', va='bottom',
                        fontsize=7)
                        # fontsize=13)

        resultsKeys = ['kickSuccessTime', 'kickFailureTime', 'collisionFailureTime']

        # series_x_offset = 0
        series_x_offset = -width*0.5
        series_num = 0
        part_rects_dict = {}
        for resultsKey in resultsKeys:
            # resultsEntry = results_table[resultsKey]
            # trials = series
            # trials = filter(lambda t:
            #     float(t['number'])==1000 and
            #     # float(t['skipFrac'])==0.1
            #     , series)
            values = map(itemgetter(resultsKey), sortedResults)
            errors = map(itemgetter(resultsKey+'Error'), sortedResults)
            # values = map(lambda v: 0 if v is None else v, values) # TODO: Handle missing values!
            # values = map(lambda v: 100*v, values) # convert to percentages
            print values
            series_rects = ax.bar(ind + series_x_offset, values, width, color=series_cols[series_num], yerr=errors)
            series_x_offset += width
            series_num += 1
            autolabel(series_rects)
            part_rects_dict[resultsKey] = series_rects

        # series_x_offset = 0
        # # series_x_offset = -width*0.5
        # series_num = 0
        # part_rects_dict = {}
        # for part_key in sorted(series_parts.keys()):
        #     series = series_parts[part_key]
        #     trials = series
        #     # trials = filter(lambda t:
        #     #     float(t['number'])==1000 and
        #     #     # float(t['skipFrac'])==0.1
        #     #     , series)
        #     # sorted_trials = sorted(trials, key=lambda x: float(x['hardNegFrac']))
        #     # values = map(itemgetter(value_key), sorted_trials)
        #     # values = map(lambda v: 0 if v is None else v, values) # TODO: Handle missing values!
        #     # values = map(lambda v: 100*v, values) # convert to percentages
        #     values = [1,2,3,4]
        #     print values
        #     series_rects = ax.bar(ind + series_x_offset, values, width, color=series_cols[series_num])
        #     series_x_offset += width
        #     series_num += 1
        #     autolabel(series_rects)
        #     part_rects_dict[part_key] = series_rects

        # add some text for labels, title and axes ticks
        # ax.set_title('Precision by feature type')
        # ax.set_ylabel('{} (\%)'.format(value_key.capitalize()))
        ax.set_ylabel('Success / Failure Time (s)')
        ax.set_xlabel('Chance Constraint. \%')
        ax.set_xticks(ind+width)
        ax.set_xticklabels(ind_labels)

        # sorted_part_keys = sorted(part_rects_dict.keys())
        legendHeadings = ['Kick Success', 'Kick Failure', 'Collision Failure']
        sorted_part_keys = resultsKeys # sorted(part_rects_dict.keys())

        # # Shrink current axis's height by 10% on the bottom
        # box = ax.get_position()
        # shrinkf = 0.8
        # ax.set_position([box.x0, box.y0,# + box.height * (1.0-shrinkf),
        #                  box.width, box.height * shrinkf])

        ax.legend([part_rects_dict[key] for key in sorted_part_keys]
        , legendHeadings
        , ncol=3
        , prop=smallFontP
        , loc='upper center'
        , bbox_to_anchor=(0.5, 1.15))
        #           fancybox=True)

        # plt.show()
        plt.savefig('results_fig_{}.pdf'.format(value_key), bbox_inches='tight')

def saveAsCsv(results_table, fname):
    with open(fname, 'w+') as results_file:
        col_names = [
             'kickSuccess',
             'kickFailure',
             'collisionFailure',
             'timeLimit',
             'targetAngleRange',
             'finishTime',
             'replanInterval',
             'searchTimeLimit',
             'numReplans',
             'chanceConstraint',
             'ballObstacleRadiusFactor',
             'ballObstacleOffsetFactor',
             'seed'
         ]
        # col_names = sorted(results_table[0].keys())
        header_row = '\t'.join(col_names)

        results_file.write(header_row + '\n')

        for result in results_table:
            row_raw = map(lambda n: result[n], col_names)
            row_raw = map(lambda n: '' if n is None else n, row_raw)
            row_str = '\t'.join(map(str, row_raw))
            results_file.write(row_str + '\n')


if __name__ == "__main__":
    # Parse arguments:
    parser = argparse.ArgumentParser(description='Plot trial results')
    parser.add_argument('resultsFile', type=str, nargs='?', default='rrbtTrials-valid.yaml', help='Results file name.')
    parser.add_argument('rrtsResultsFile', type=str, nargs='?', default='rrtsTrials-valid.yaml', help='Results file name.')
    args = parser.parse_args()

    # Load results from file:
    trialResults = loadResultsTable(args.resultsFile)
    rrtsTrialResults = loadResultsTable(args.rrtsResultsFile)

    saveAsCsv(trialResults, 'results-rrbt.tsv')
    saveAsCsv(rrtsTrialResults, 'results-rrts.tsv')
    sys.exit(0)

    # Group the chanceConstraint into bins:
    for res in trialResults:
        percentile = res['chanceConstraint']
        if percentile >= 0.9:
            res['chanceConstraint'] = 0.95
        elif percentile >= 0.8:
            res['chanceConstraint'] = 0.85
        elif percentile >= 0.7:
            res['chanceConstraint'] = 0.75
        else:
            res['chanceConstraint'] = 0.65

    chanceParts = partitionOn('chanceConstraint', trialResults)
    chanceParts['RRT*'] = rrtsTrialResults

    # { kickSuccess: 0
    # , kickFailure: 0
    # , initialState: [0.982621,0.688455,0.761885]
    # , finalState: [1.05826,0.861568,-2.83624]
    # , collisionFailure: 1
    # , timeLimit: 60
    # , targetAngleRange: 1.5708
    # , finishTime: 4.4
    # , replanInterval: 5
    # , numReplans: 1
    # , chanceConstraint: 0.688884
    # , seed: 42770935
    # },
    resultsTable = {}
    timeTable = {}
    for key in chanceParts.keys():
        part = chanceParts[key]
        partSize = len(part)

        if key != 'RRT*':
            key = int(round(float(key)*100))

        numKickSuccess = len(filter(lambda t: int(t['kickSuccess']) == 1, part))
        numKickFailure = len(filter(lambda t: int(t['kickFailure']) == 1, part))
        numCollisionFailure = len(filter(lambda t: int(t['collisionFailure']) == 1, part))

        partStats = {}
        partStats['kickSuccessRate'] = 100*(float(numKickSuccess) / partSize)
        partStats['kickFailureRate'] = 100*(float(numKickFailure) / partSize)
        partStats['collisionFailureRate'] = 100*(float(numCollisionFailure) / partSize)
        partStats['chanceConstraint'] = key
        partStats['sampleCount'] = partSize

        resultsTable[key] = partStats

        # Calculate times:
        kickSuccessTimes = map(lambda t: float(t['finishTime']), filter(lambda t: int(t['kickSuccess']) == 1, part))
        kickFailureTimes = map(lambda t: float(t['finishTime']), filter(lambda t: int(t['kickFailure']) == 1, part))
        collisionFailureTimes = map(lambda t: float(t['finishTime']), filter(lambda t: int(t['collisionFailure']) == 1, part))

        def popMean(lst):
            if len(lst) == 0:
                return 0
            return sum(lst)/float(len(lst))
        def sampleVariance(lst):
            if len(lst) <= 1:
                return 0
            avg = popMean(lst)
            dev = map(lambda x: (x - avg)*(x - avg), lst)
            return sum(dev)/float(len(lst) - 1)
        def sampleStdDev(lst):
            return math.sqrt(sampleVariance(lst))

        partTimes = {}
        partTimes["kickSuccessTime"] = popMean(kickSuccessTimes)
        partTimes["kickSuccessTimeError"] = sampleStdDev(kickSuccessTimes)
        partTimes["kickFailureTime"] = popMean(kickFailureTimes)
        partTimes["kickFailureTimeError"] = sampleStdDev(kickFailureTimes)
        partTimes["collisionFailureTime"] = popMean(collisionFailureTimes)
        partTimes["collisionFailureTimeError"] = sampleStdDev(collisionFailureTimes)

        timeTable[key] = partTimes

    print 'resultsTable:', resultsTable
    print 'timeTable:', timeTable

    plotFailureSuccessRates(resultsTable)
    plotResultsTable(timeTable)
