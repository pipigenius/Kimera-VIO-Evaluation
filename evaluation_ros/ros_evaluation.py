import evaluation, evaluation.tools

import argcomplete
import sys
import yaml
import os

def run_ros(args):
    from evo.tools import log
    from evo.tools.settings import SETTINGS

    # Get experiment information from yaml file.
    experiment_params = yaml.load(args.experiments_path)

    results_dir = os.path.expandvars(experiment_params['results_dir'])
    params_dir = os.path.expandvars(experiment_params['params_dir'])
    dataset_dir = os.path.expandvars(experiment_params['dataset_dir'])
    datasets_to_run = experiment_params['datasets_to_run']

    # Here the results has already been run with the bash scripts
    successful_run = True;
    for dataset in datasets_to_run:
        print("Run dataset:", dataset['name'])
        pipelines_to_run = dataset['pipelines']
        if not evaluation.run_dataset(results_dir, params_dir, dataset_dir, dataset, "",
                           False, args.analyse_vio,
                           args.plot, args.save_results,
                           args.save_plots, args.save_boxplots,
                           pipelines_to_run,
                           dataset['initial_frame'],
                           dataset['final_frame'],
                           dataset['discard_n_start_poses'],
                           dataset['discard_n_end_poses']):
            successful_run = False
    return successful_run

if __name__ == '__main__':
    parser = evaluation.parser()
    args = parser.parse_args(['test_offline.yaml', '-a',
                              '--save_results', '--save_plots', '--save_boxplots'])
    run_ros(args)
