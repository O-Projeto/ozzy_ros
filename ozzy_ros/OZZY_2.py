import google.generativeai as genai
import pyttsx3

def main():
    assistente_falante = True
    ligar_microfone = True

    genai.configure(api_key="AIzaSyDetqlvGmCYU-hIgX6FEBCMMYW9BlM1Mcc")
    for m in genai.list_models():
        if 'generateContent' in m.supported_generation_methods:
            print(m.name)

    model = genai.GenerativeModel('gemini-pro')
    chat = model.start_chat(history=[])

    ### configura voz
    if assistente_falante:
        engine = pyttsx3.init()

        voices = engine.getProperty('voices')
        engine.setProperty('rate', 200) # velocidade 120 = lento

        voz = 0
        engine.setProperty('voice', voices[voz].id)

    if ligar_microfone:
        import speech_recognition as sr  # pip install SpeechRecognition
        r = sr.Recognizer()
        mic = sr.Microphone()

    bem_vindo = "# Bem Vindo ao Assistente Mil Grau com Gemini AI #"
    print("")
    print(len(bem_vindo) * "#")
    print(bem_vindo)
    print(len(bem_vindo) * "#")
    print("###   Digite 'desligar' para encerrar    ###")
    print("")

    while True:
        if ligar_microfone:
            with mic as fonte:
                r.adjust_for_ambient_noise(fonte)
                print("Fale alguma coisa (ou diga 'desligar')")
                audio = r.listen(fonte)
                print("Enviando para reconhecimento")
                try:
                    texto = r.recognize_google(audio, language="pt-BR")
                    print("Você disse: {}".format(texto))
                except Exception as e:
                    print("Não entendi o que você disse. Erro", e)
                    texto = ""
        else:
            texto = input("Escreva sua mensagem (ou #sair): ")

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