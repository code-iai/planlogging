#!/bin/bash

source ~/.bashrc
PLANLOGGER_DIR=`rospack find planlogger`
cd ${PLANLOGGER_DIR}
rm -Rf packaging
mkdir -p packaging

mongoexport -d db_exp_log -c col_exp_log -o packaging/planlog.json
mongoexport -d roslog -c tf -o packaging/tf.json
mongoexport -d roslog -c uima_trigger -o packaging/uima_trigger.json
mongoexport -d roslog -c uima_uima_results -o packaging/uima_uima_results.json

cp -r experiments/current-experiment/* packaging
./makedot.sh

DATE_STRING=`date +"%Y-%m-%d-%H-%M-%S"`
tar -czf log-packaged-${DATE_STRING}.tar.gz packaging/*
