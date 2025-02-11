# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import ActionExecuted, UserUtteranceReverted, Restarted
import pyAgrum as gum

import rospy
from std_msgs.msg import String, Bool
import pyttsx3
import yaml
import os


class MentorPepComponent:
    def __init__(self):
        self.NETWORK_STRUCTURE = {
        'parent_nodes': {
            'tech_affinity': ['low', 'medium', 'high'],
            'creativity': ['low', 'medium', 'high'],
            'leadership': ['low', 'medium', 'high'],
            'work_life_balance': ['low', 'medium', 'high']
        },
        'career_nodes': {
            'data_scientist': ['unsuitable', 'suitable', 'highly_suitable'],
            'software_engineer': ['unsuitable', 'suitable', 'highly_suitable'],
            'ux_designer': ['unsuitable', 'suitable', 'highly_suitable'],
            'hr_manager': ['unsuitable', 'suitable', 'highly_suitable']
        },
        'dependencies': [
            ('tech_affinity', 'data_scientist'),
            ('creativity', 'data_scientist'),
            ('tech_affinity', 'software_engineer'),
            ('work_life_balance', 'software_engineer'),
            ('creativity', 'ux_designer'),
            ('work_life_balance', 'ux_designer'),
            ('leadership', 'hr_manager'),
            ('work_life_balance', 'hr_manager')
        ]
        }
        self.bn = gum.BayesNet('MentorPep')
        self._setup_network()

    def _setup_network(self):
        for node, values in self.NETWORK_STRUCTURE['parent_nodes'].items():
            self.bn.add(gum.LabelizedVariable(node, node, values))
        
        for node, values in self.NETWORK_STRUCTURE['career_nodes'].items():
            self.bn.add(gum.LabelizedVariable(node, node, values))
        
        for parent, child in self.NETWORK_STRUCTURE['dependencies']:
            self.bn.addArc(parent, child)

        self._initialize_cpts()

    def _initialize_cpts(self):
        for node in self.NETWORK_STRUCTURE['parent_nodes']:
            self.bn.cpt(node).fillWith([1/3, 1/3, 1/3])

        basic_career_cpt = [0.7, 0.2, 0.1, 0.5, 0.3, 0.2, 0.3, 0.4, 0.3,
                           0.5, 0.3, 0.2, 0.3, 0.4, 0.3, 0.2, 0.3, 0.5,
                           0.3, 0.3, 0.4, 0.2, 0.3, 0.5, 0.1, 0.2, 0.7]

        for career in self.NETWORK_STRUCTURE['career_nodes']:
            self.bn.cpt(career).fillWith(basic_career_cpt)
    
    def get_career_recommendations(self, answers):

        ie = gum.LazyPropagation(self.bn)
        ie.setEvidence(answers)
        ie.makeInference()

        recommendations = []
        for career in self.NETWORK_STRUCTURE['career_nodes']:
            probs = ie.posterior(career)
            recommendations.append((career, float(probs[2])))
        
        return sorted(recommendations, key=lambda x: x[1], reverse=True)


class ActionGoodbye(Action):
    def name(self) -> str:
        return "action_stop"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):

        return [ActionExecuted("action_restart")]


class ActionRecommendCareers(Action):
    def name(self) -> Text:
        return "action_recommend_careers"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> List[Dict]:
        job_descriptions = {
            'data_scientist':  "A Data Scientist analyzes and\
                interprets complex data to help businesses make informed decisions.\
                They use tools like Python, R, and SQL to process data and build predictive models.\
                Common industries include tech, healthcare, finance, and e-commerce.",
            'software_engineer': "A Software Engineer designs, develops, and maintains software systems.\
                They often write code in languages like Java, Python, or JavaScript.\
                You could work on web apps, mobile apps, or even AI systems.\
                This role suits people with a strong tech affinity and a passion for building solutions.?",
            'ux_designer': "UX Designers often have a balanced workload compared to other tech roles.\
                Work involves user research, design, and testing, which is typically project-based.\
                Deadlines can sometimes lead to busy periods, but overall, the industry values creativity\
                and a healthy work environment.",
            'hr_manager': "An HR Manager focuses on recruitment, employee engagement, and organizational development. \
                They often work on policy-making and creating a positive workplace culture. Do you need any informations about other careers?"
        }
        answers = {}
        answers['tech_affinity'] = self.interpret_tech_affinity(tracker.get_slot("tech_affinity"))
        answers['creativity'] = self.interpret_creativity(tracker.get_slot("creativity"))
        answers['leadership'] = self.interpret_leadership(tracker.get_slot("leadership"))
        answers['work_life_balance'] = self.interpret_work_life_balance(tracker.get_slot("work_life_balance"))
        
        mentor = MentorPepComponent()
        recomendation = mentor.get_career_recommendations(answers)
        dispatcher.utter_message(text=f"Based on your preferences, I recommend considering a career as a {recomendation[0][0].replace('_', ' ').title()}.\n \n {job_descriptions[recomendation[0][0]]} \n Thank you for using our career recommendation service. ")
        return []

    def interpret_tech_affinity(self, answer: str) -> str:
        answer = answer.lower()
        high_keywords = ["love", "enjoy", "absolutely", "really", "yes", "comfortable", "exciting", "very familiar", "favorite", "highly", "my thing","extremely","passion" ]

        low_keywords = ["not", "dislike", "not at all", "no", "don't like", "limited"]

        if any(keyword in answer for keyword in low_keywords):
            return "low"
        
        elif any(keyword in answer for keyword in high_keywords):
            return "high"
        else:
            return "medium"
        
    def interpret_creativity(self, answer: str) -> str:
        answer = answer.lower()
        high_keywords = ["very important", "essential", "thrive", "critical", "must", "vital", "crucial", "key"]

        low_keywords = ["not","not important", "doesn't matter", "prefer not", "don't need", "small role", "rely more on"]

        if any(keyword in answer for keyword in low_keywords):
            return "low"
        
        elif any(keyword in answer for keyword in high_keywords):
            return "high"
        else:
            return "medium"

    def interpret_leadership(self, answer: str) -> str:
        answer = answer.lower()
        high_keywords = ["lead", "leading", "leader", "charge", "leadership roles", "suits me","motivating", "guiding"]

        low_keywords = ["rather than leading", "team", "part of a team", "don't like leading", "prefer not to lead", "work under a leader", "team player", "collaborating", "supporting", "contributing"]

        
        if any(keyword in answer for keyword in low_keywords):
            return "low"
        elif any(keyword in answer for keyword in high_keywords):
                return "high"
        else:
            return "medium"

    def interpret_work_life_balance(self, answer: str) -> str:
        answer = answer.lower()
        high_keywords = ["very","a lot","very important", "essential", "balance", "free time", "life", "prioritize personal life", "healthy balance"]

        low_keywords = ["flexible ","work is priority", "work comes first", "no time off", "don't mind working late", "not a concern", "not", "work"]

        if any(keyword in answer for keyword in low_keywords):
            return "low"
        
        elif any(keyword in answer for keyword in high_keywords):
            return "high"
        else:
            return "medium"
##############################################################################

class Ros1Publisher:
    def __init__(self):
        # Initialize the ROS node if not already initialized
        if not rospy.core.is_initialized():
            rospy.init_node('rasa_publisher', anonymous=True, disable_signals=True)
        self.publisher = rospy.Publisher('pepper_commands', String, queue_size=10)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        rospy.loginfo(f"Published command: {command}")


class ActionGreetWithAnimation(Action):
    def name(self) -> str:
        return "action_greet_with_animation"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        # Create a publisher instance
        publisher = Ros1Publisher()

        # Publish the "wave" command once
        publisher.publish_command("wave")

        # Respond to the user
        # dispatcher.utter_message(text="Hello! Nice to meet you!")
        return []


class ActionGoodbyeWithAnimation(Action):
    def name(self) -> str:
        return "action_goodbye_with_animation"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        # Create a publisher instance
        publisher = Ros1Publisher()

        # Publish the "thumbs_up" command once
        publisher.publish_command("wave")
        
        # Respond to the user
        # dispatcher.utter_message(text="Goodbye! Take care!")
        return []
    
class ActionIdleWithAnimation(Action):
    def name(self) -> str:
        return "action_idle_with_animation"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        # Create a publisher instance
        publisher = Ros1Publisher()

        # Publish the "thumbs_up" command once
        publisher.publish_command("present_both_hands")
        
        # Respond to the user
        # dispatcher.utter_message(text="Goodbye! Take care!")
        return []
    
class ActionThumbsUpWithAnimation(Action):
    def name(self) -> str:
        return "action_thumbsup_with_animation"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        # Create a publisher instance
        publisher = Ros1Publisher()

        # Publish the "thumbs_up" command once
        publisher.publish_command("thumbs_up")
        
        # Respond to the user
        # dispatcher.utter_message(text="Goodbye! Take care!")
        return []
    
###################################
class RosTTS:
    def __init__(self):
        # Initialize ROS publisher
        if not rospy.core.is_initialized():
            rospy.init_node("rasa_actions", anonymous=True, disable_signals=True)
        self.publisher = rospy.Publisher("/tts_commands", String, queue_size=10)

    def publish(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        rospy.loginfo(f"Published TTS command: {text}")


class ActionGreetSpeech(Action):
    def name(self) -> str:
        return "action_greet_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        
        response = "Hi! I am MentorPep, your career recommendation assistant. \
            Would you like to answer me a few questions to estimate the best career path for you"
        tts = RosTTS()
        tts.publish(response)
        return []


class ActionGoodbyeSpeech(Action):
    def name(self) -> str:
        return "action_goodbye_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "Goodbye! Take care"

        tts = RosTTS()
        tts.publish(response)
        return []
    

class ActionConfirmStart(Action):
    def name(self) -> str:
        return "action_confirm_start_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "Great! Let's begin. I'll ask you a few questions about your preferences."

        tts = RosTTS()
        tts.publish(response)
        return []

class ActionDenyStart(Action):
    def name(self) -> str:
        return "action_deny_start_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "No problem! You can try later if you change your mind."

        tts = RosTTS()
        tts.publish(response)
        return []

class ActionAskTechAffinity(Action):
    def name(self) -> str:
        return "action_ask_tech_affinity_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "How familiar are you with modern technologies and do you like to work with them?"

        tts = RosTTS()
        tts.publish(response)
        return []

class ActionAskCreativity(Action):
    def name(self) -> str:
        return "action_ask_creativity_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "How important is creativity in your work?"

        tts = RosTTS()
        tts.publish(response)
        return []

class ActionAskLeadership(Action):
    def name(self) -> str:
        return "action_ask_leadership_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "Do you prefer leadership roles or being part of a team?"

        tts = RosTTS()
        tts.publish(response)
        return []

class ActionAskWorkLifeBalance(Action):
    def name(self) -> str:
        return "action_ask_work_life_balance_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "How crucial is work-life balance for you?"

        tts = RosTTS()
        tts.publish(response)
        return []

class ActionSubmitForm(Action):
    def name(self) -> str:
        return "action_submit_form_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "Thanks for sharing your preferences!"

        tts = RosTTS()
        tts.publish(response)
        return []

class ActionAbusiveLanguage(Action):
    def name(self) -> str:
        return "action_abusive_language_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "Please keep the conversation respectful and appropriate."

        tts = RosTTS()
        tts.publish(response)
        return []

class ActionOutOfTopic(Action):
    def name(self) -> str:
        return "action_out_of_topic_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "It seems like we're getting off topic. Do you want to stop?"

        tts = RosTTS()
        tts.publish(response)
        return []

class ActionIamABot(Action):
    def name(self) -> str:
        return "action_iamabot_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "I am a bot, powered by Rasa."

        tts = RosTTS()
        tts.publish(response)
        return []
    
class ActionRecommendCareer(Action):
    def name(self) -> str:
        return "action_recommend_career_speech"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        response = "According to your responses I think this career suites you well"

        tts = RosTTS()
        tts.publish(response)
        return []

###########################################
    

class NameUpdater:
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node("rasa_action_server", anonymous=True, disable_signals=True)
        rospy.Subscriber("/detected_faces", String, self.update_name)
        self.current_name = "Unknown"

    def update_name(self, msg):
        self.current_name = msg.data
        rospy.loginfo(f"Updated name: {self.current_name}")

# Instantiate NameUpdater globally
NAME_UPDATER = NameUpdater()

class ActionGetName(Action):
    def name(self) -> str:
        return "action_get_name"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        name = NAME_UPDATER.current_name
        response = f"Hello, {name}!" if name != "Unknown" else "Hello, stranger!"
        dispatcher.utter_message(text=response)
        return []

class ConversationResetter:
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node("rasa_action_server", anonymous=True, disable_signals=True)
        rospy.Subscriber("/reset_conversation", Bool, self.reset_conversation)
        self.reset_flag = False

    def reset_conversation(self, msg):
        if msg.data:
            self.reset_flag = True
            rospy.loginfo("Reset flag set to True")
            # return Restarted()

# Instantiate ConversationResetter globally
CONVERSATION_RESETTER = ConversationResetter()

class ActionResetConversation(Action):
    def name(self) -> str:
        return "action_reset_conversation"

    def run(self, dispatcher: CollectingDispatcher, tracker, domain):
        rospy.loginfo("Restarting Conversation")
        # CONVERSATION_RESETTER = ConversationResetter()
        if CONVERSATION_RESETTER.reset_flag:
            CONVERSATION_RESETTER.reset_flag = False
            dispatcher.utter_message(text="Conversation reset.")
            return [Restarted()]
        return []
