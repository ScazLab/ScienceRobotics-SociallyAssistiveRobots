#!/usr/bin/env python
import json
from subprocess import call


class update_ips(object):
    '''
    update ip in game config files & reset session data and performance logs
    '''

    def __init__(self):
        self.game_name2filepath_dict = {}
        self.game_name2folder_dict = {}

        self.IP = '192.168.2.3' # PC's IP


    def run(self):
        with open('/home/sar/catkin_ws/src/sar_core/config/session_manager_config.json') as config_file:
            configurations = json.load(config_file)

        self.game_name2filepath_dict['storytelling'] = configurations['game_social_stories_filepath']
        self.game_name2filepath_dict['rocket_barrier'] = configurations['game_rocket_barrier_filepath']
        self.game_name2filepath_dict['house_perspective_taking'] = configurations['game_house_perspective_taking_filepath']
        self.game_name2filepath_dict['train_sequencing'] = configurations['game_train_sequencing_filepath']

        for key in self.game_name2filepath_dict:
            path = self.game_name2filepath_dict[key]
            path_to_folder = path.split('.')[0]
            self.game_name2folder_dict[key] = path_to_folder

        self.update_yale_game('rocket_barrier')
        self.update_yale_game('house_perspective_taking')
        self.update_yale_game('train_sequencing')

        # update MIT game ip
        self.update_mit_game()

        # reset session data
        self.reset_session_data()

        # delete performance log
        call(["rm", "/home/sar/catkin_ws/src/sar_core/config/performance_log.json"])

        # delete vja log
        call(["rm", "/home/sar/catkin_ws/src/sar_core/logs/vja_log.json"])


    def update_yale_game(self, game_name):
        with open(self.game_name2folder_dict[game_name].encode('ascii', 'ignore')+'_Data/Resources/yurp_config.txt', 'w+') as f:
            f.write('{\n')
            f.write('"server": "' + self.IP + '",\n')
            f.write('"port": "9090",\n')
            f.write('"log_debug_to_ros": true\n')
            f.write('}')


    def update_mit_game(self):
        with open(self.game_name2folder_dict['storytelling']+'_Data/Resources/opal_config.txt', 'w+') as f:
            f.write('{\n')
            f.write('"server": "' + self.IP + '",\n')
            f.write('"port": "9090",\n')
            f.write('"opal_action_topic": "/sar/opal_action",\n')
            f.write('"opal_audio_topic": "/sar/opal_audio",\n')
            f.write('"opal_command_topic": "/sar/opal_command",\n')
            f.write('"opal_log_topic": "/sar/opal_tablet",\n')
            f.write('"opal_scene_topic": "/sar/opal_tablet_scene",\n')
            f.write('"toucan": false,\n')
            f.write('"log_debug_to_ros": true\n')
            f.write('}')


    def reset_session_data(self):
        with open('/home/sar/catkin_ws/src/sar_core/logs/session_data.json', 'w+') as f:
            f.write('{"sessions": []}')


if __name__ == "__main__":
    uips = update_ips()
    uips.run()
