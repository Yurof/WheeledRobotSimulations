for f in results/individuals/NoveltyFitness/selected/*; do
    NAME=${f#results/individuals/}
    echo ${NAME::-4}
    python Simulationfastsim/main.py --display False --file_name ${NAME::-4} --criteria 'NoveltyFitness'
    python pyBullet/main.py --file_name ${NAME::-4} --criteria 'NoveltyFitness'
done

