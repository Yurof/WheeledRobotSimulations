rm -f results/individuals/NoveltyFitness/results.csv
for f in results/individuals/NoveltyFitness/FastSim/*; do
    NAME=${f#results/individuals/NoveltyFitness/FastSim/}
    NAME=${NAME::-4}
    python error.py --file_name $NAME --criteria 'NoveltyFitness'
done