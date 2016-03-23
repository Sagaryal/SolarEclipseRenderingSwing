/**
@file
@author Benny Bobaganoosh <thebennybox@gmail.com>
@section LICENSE

Copyright (c) 2014, Benny Bobaganoosh
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import java.io.IOException;

/**
 * The sole purpose of this class is to hold the main method.
 *
 * Any other use should be placed in a separate class
 */
public class Main
{
	// Lazy exception handling here. You can do something more interesting 
	// depending on what you're doing
	public static void main(String[] args) throws IOException
	{
		Display display = new Display(1366, 840, "Software Rendering");
		RenderContext target = display.GetFrameBuffer();
		
		Stars3D stars = new Stars3D(2000);

		Bitmap earth = new Bitmap("./res/earth.bmp");
		Bitmap moon = new Bitmap("./res/moon.png");
		Bitmap sun = new Bitmap("./res/sun.bmp");
		
		Mesh earthMesh = new Mesh("./res/earth.obj");
		Transform earthTransform = new Transform(new Vector4f(2,0.0f,0.0f));
		earthTransform.scale(new Vector4f(0.8f, 0.8f, 0.8f));
		earthTransform = earthTransform.Rotate(new Quaternion(new Vector4f(0,0,1), (float) Math.toRadians(-20)));
		earthTransform = earthTransform.Rotate(new Quaternion(new Vector4f(1,0,0), (float) Math.toRadians(25)));
		
		
		Mesh moonMesh = new Mesh("./res/moon.obj");
		Transform moonTransform1 = new Transform(new Vector4f(2.0f,0.0f,0.0f));
		moonTransform1.scale(new Vector4f(0.5f, 0.5f, 0.5f));
	
		Transform moonTransform2 = new Transform(new Vector4f(0,0.0f,1.5f));
		//moonTransform2.scale(new Vector4f(0.5f, 0.5f, 0.5f));
		
		
		Mesh sunMesh = new Mesh("./res/sun.obj");
		Transform sunTransform = new Transform(new Vector4f(-0.5f,0.0f,0.0f));
		//sunTransform.scale(new Vector4f(0.5f, 0.5f, 0.5f));
		//Mesh monkeyMesh = new Mesh("./res/smoothMonkey0.obj");
		//Transform monkeyTransform = new Transform(new Vector4f(0,0.0f,3.0f));

		//Mesh terrainMesh = new Mesh("./res/terrain2.obj");
		//Transform terrainTransform = new Transform(new Vector4f(0,-1.0f,0.0f));

		Camera camera = new Camera(new Matrix4f().InitPerspective((float)Math.toRadians(70.0f),
					   	(float)target.GetWidth()/(float)target.GetHeight(), 0.1f, 1000.0f));
		camera.Move(new Vector4f(0, 0, -3, 0), 1);
		
		
		float rotCounter = 0.0f;
		
		long previousTime = System.nanoTime();
		while(true)
		{
	
			long currentTime = System.nanoTime();
			float delta = (float)((currentTime - previousTime)/1000000000.0);
			previousTime = currentTime;

			camera.Update(display.GetInput(), delta);
			Matrix4f vp = camera.GetViewProjection();

			earthTransform = earthTransform.Rotate(new Quaternion(new Vector4f(0,1,0), -delta));
			moonTransform1 = moonTransform1.Rotate(new Quaternion(new Vector4f(0,1,0), -delta));
			moonTransform2.Rotate(new Quaternion(new Vector4f(0,1,0), delta));
			
			
			
			Matrix4f translationMatrix = new Matrix4f().InitTranslation(moonTransform2.GetPos().GetX(), moonTransform2.GetPos().GetY(), 
					moonTransform2.GetPos().GetZ());
			
			Matrix4f rotationMatrix = moonTransform2.GetRot().ToRotationMatrix();
			//Matrix4f scaleMatrix = new Matrix4f().InitScale(moonTransform2.GetScale().GetX(),
				//	moonTransform2.GetScale().GetY(), moonTransform2.GetScale().GetZ());

			Matrix4f transformedMoon2 = translationMatrix.Mul(rotationMatrix);
			//moonTransform1 = moonTransform1.SetPos(new Vector4f(0.0f,1.0f,0.0f));
			//moonTransform1 = moonTransform1.Rotate(new Quaternion(new Vector4f(0,1,0), -delta));
			
			
			Matrix4f transformedMoon1 = moonTransform1.GetTransformation();
			target.Clear((byte)0x00);
			stars.Render(target);
			target.ClearDepthBuffer();
			earthMesh.Draw(target, vp, earthTransform.GetTransformation(), earth);
			moonMesh.Draw(target, vp, transformedMoon1.Mul(transformedMoon2), moon);
			sunMesh.Draw(target, vp, sunTransform.GetTransformation(), sun);

			display.SwapBuffers();
			
			//moonTransform2 = new Transform(moonTransfor)
			
			
		}
	}
}
