'''
    File name: agent.py
    Author: Mohamed S. Talamali and Andreagiovanni Reina
            University of Sheffield, UK
    Date created: October 2018 (By Francesco Cancianni)
    Date last modified: October 2020 (By Mohamed S. Talamali)
'''
class Msg:
    def __init__(self, m_id, opinion, quality, totemPos,disappearanceTime):
        self.id = m_id
        self.opinion = opinion
        self.quality = quality
        self.visible = True
        self.active = True
        self.totemPos = totemPos
        self.disappearanceTime = disappearanceTime
