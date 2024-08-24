import ollama
import cv2

# Take user input
# user_prompt = input("Enter your prompt: ")
# Capture picture from front camera
# camera = cv2.VideoCapture(0)
# ret, frame = camera.read()

# Generate response using ollama
response = ollama.chat(model='bot_bheem',
    messages=[
		{
			'role': 'user',
			'content': 'Describe this image:',
			'images': ['image.png']
		}
	],
    stream=False,
)



# # Save the captured image
# cv2.imwrite("front_camera_image.jpg", frame)

# # Display the response and the image
print(response['message']['content'])
# cv2.imshow("Front Camera Image", frame)
# cv2.waitKey(0)
# cv2.destroyAllWindows()