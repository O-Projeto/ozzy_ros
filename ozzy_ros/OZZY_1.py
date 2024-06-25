import google.generativeai as genai
import speech_recognition as sr
import pyttsx3

def ouvir_microfone():
     #Habilita o microfone para ouvir o usuario
     microfone = sr.Recognizer()
     with sr.Microphone() as source:
          #Chama a funcao de reducao de ruido disponivel na speech_recognition
          microfone.adjust_for_ambient_noise(source)
          #Avisa ao usuario que esta pronto para ouvir
          print("Diga alguma coisa: ")
          #Armazena a informacao de audio na variavel
          audio = microfone.listen(source)
          #Passa o audio para o reconhecedor de padroes do speech_recognition
          frase = microfone.recognize_google(audio,language='pt-BR')
          #Após alguns segundos, retorna a frase falada
          print("Você disse: " + frase)
    #Caso nao tenha reconhecido o padrao de fala, exibe esta mensagem

     return frase


def main():
    assistente_falante = True

    genai.configure(api_key="AIzaSyDetqlvGmCYU-hIgX6FEBCMMYW9BlM1Mcc")
    for m in genai.list_models():
        if 'generateContent' in m.supported_generation_methods:
            print(m.name)

    model = genai.GenerativeModel('gemini-1.5-flash')
    chat = model.start_chat(history=[])

    ### configura voz
    if assistente_falante:
        engine = pyttsx3.init()

        voices = engine.getProperty('voices')
        engine.setProperty('rate', 200) # velocidade 120 = lento

        voz = 0
        engine.setProperty('voice', voices[voz].id)



    bem_vindo = "# Bem Vindo ao Assistente Mil Grau com Gemini AI #"
    print("")
    print(len(bem_vindo) * "#")
    print(bem_vindo)
    print(len(bem_vindo) * "#")
    print("###   Digite 'desligar' para encerrar    ###")
    print("")

    while True:
        texto = ouvir_microfone()

        if texto.lower() == "desligar":
            break

        response = chat.send_message(texto)
        print("Gemini:", response.text, "\n")

        if assistente_falante:
            engine.say(response.text)
            engine.runAndWait()

    print("Encerrando Chat")

if __name__ == '__main__':
    main()
