python -m scoop fastsim/controllers/novelty/nsga2.py --env='maze' --nb_gen=200 --mu=100 --lambda_=100 --variant='NS' --file_name='maze_ns11'
python -m scoop fastsim/controllers/novelty/nsga2.py --env='maze' --nb_gen=200 --mu=100 --lambda_=100 --variant='FIT+NS' --file_name='maze_nsfit11'
python -m scoop fastsim/controllers/novelty/nsga2.py --env='maze' --nb_gen=200 --mu=100 --lambda_=100 --variant='FIT' --file_name='maze_fit11'
